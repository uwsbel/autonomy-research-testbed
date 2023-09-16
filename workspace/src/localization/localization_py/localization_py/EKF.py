import numpy as np
import math
import yaml


class EKF:
    """A basic Extended Kalman Filter implementation.
    
    This currently uses the 4DOF motion model. The Motion Model inputs are throttle and steering, and the observation model uses GPS and Magnetometer as sensors. States are cartesian coordinates x, y, heading theta, and velocity v.

    Attributes:
        c_1: The motor resistence torque linear component.
        c_0: The motor resistence torque constant component.
        l: The length of the vehicle.
        r_wheel: The radius of the wheel.
        i_wheel: the intertia of the wheel.
        gamma: The gear ratio.
        tau_0: The stalling torque for the motor torque model.
        omega_0: The maximum no-load speed for the motor torque model.
        df_1_dv: The derivative of the scaled motor torque relative to the velocity of the vehicle.
        Q: The covariane matrix for our states.
        R: The covariance matrix for our sensors.
        P: The predicted covariance estimate.
        dt: The timestep for the filter.
    """

    def __init__(self, dt):
        """Initialize the EKF.

        Initialize each of the global variables for the EKF. Load the dynamics parameters for the vehicle from an external YAML file.

        Args:
            dt: The timestep at which this filter will operate at. 
        """

        with open('/home/art/art/workspace/src/localization/localization_py/localization_py/4DOF_dynamics.yml', 'r') as yaml_file:
            config = yaml.safe_load(yaml_file)
        self.dt = dt
        #vehicle parameters:
        self.c_1 = config['c_1']
        self.c_0 = config['c_0']
        self.l = config['l']
        self.r_wheel = config['r_wheel']
        self.i_wheel = config['i_wheel']
        self.gamma = config['gamma']
        self.tau_0 = config['tau_0']
        self.omega_0 = config['omega_0']
        self.df_1_dv = -self.tau_0/(self.omega_0*self.r_wheel*self.gamma)

        self.Q = np.diag([
            config['Q1'],  # variance of location on x-axis
            config['Q1'],  # variance of location on y-axis
            np.deg2rad(config['Q3']),  # variance of yaw angle
            config['Q4'] # variance of velocity
        ]) ** 2  # predict state covariance
        # Observation x,y position covariance
        self.R = np.diag([config['R1'], config['R1'], config['R3']]) ** 2
        self.P = np.eye(4)

    def motion_model(self, x, u):
        """The 4-DOF motioin model.
        
        This motion model is described in more detail here:https://sbel.wisc.edu/wp-content/uploads/sites/569/2023/06/TR-2023-06.pdf.

        Args:
            x: The state vector of the vehicle, consisting of cartesian coordinates x, y, heading theta, and velocity v.
            u: The control input vector for the vehicle, consisting of throttle alpha, and steering delta.
        
        Returns:
            An updated state for the vehicle based off of the 4-DOF model propogation. This is always a 4 dimensional vector.
        """

        x[0,0] = x[0,0]+math.cos(x[2,0])*self.dt*x[3,0]
        x[1,0] = x[1,0]+math.sin(x[2,0])*self.dt*x[3,0]
        x[2,0] = x[2,0]+self.dt*x[3,0]*math.tan(u[1,0])/self.l
        f = self.tau_0*u[0,0]-self.tau_0*x[3,0]/(self.omega_0*self.r_wheel*self.gamma)

        x[3,0] = x[3,0]+ \
            self.dt*((self.r_wheel*self.gamma)/self.i_wheel)*(f-(x[3,0]*self.c_1)/(self.r_wheel*self.gamma)-self.c_0)

        return x

    def calc_F(self, v, theta, delta):
        """
        The State Transition Matrix.

        This is a linearization of the above motion model, used for updating the predicted covariance matrix.This funciton returns the state transition matrix based off of the states and inputs that appear in partial derivatives.

        Args:
            v: The velocity of the vehicle.
            theta: The heading of the vehicle.
            delta: the steering angle of the vehicle.

        Returns:
            The linearized motion model - Jacobian for the equations described above.
        """

        F = np.array([
            [self.dt*1.0, 0.0, - self.dt*v *
                math.sin(theta), self.dt*math.cos(theta)],
            [0.0, 1.0, self.dt*v *
                math.cos(theta), self.dt*math.sin(theta)],
            [0.0, 0.0, self.dt*1.0, self.dt*np.tan(delta)/self.l],
            [0.0, 0.0, 0.0,
            self.dt*(1+(self.r_wheel*self.gamma/(self.i_wheel))*(-(self.tau_0/(self.omega_0*self.r_wheel*self.gamma))-(self.c_1/(self.r_wheel*self.gamma))))]
         ])

        return F

    def angle_diff(self,ref, act):
        """Computes the difference between two angles.

        Computes the difference between two angles in radians using modular subtraction. This solves the issue when we have a positive and negative angle, both with very large magnitude.

        Args:
            ref: The reference angle in radians.
            act: the actual angle in radians.
        
        Returns:
            The difference between the two angles, using modular arithmatic. For example, an input of {pi, pi/2} will return pi/2. An input of {-pi+epsilon, pi-epsilon} will return 2 epsilon.
        """
        
        if( (ref>0 and act>0) or (ref<=0 and act <=0)):
            err_theta = ref-act
        elif( ref<=0 and act > 0):
            if(abs(ref-act)<abs(2*np.pi+ref-act)):
                err_theta = -abs(act-ref)
            else:
                err_theta = abs(2*np.pi + ref- act)
        else:
            if(abs(ref-act)<abs(2*np.pi-ref+act)):
                err_theta = abs(act-ref)
            else:
                err_theta = -abs(2*np.pi-ref+act)
        return err_theta

    def predict(self, x, u):
        """The preidction phase of the EKF.

        Updates the state based off of the motion model, and ensures that angles are in the range [-pi, pi]. Updates the linearized motion model, and the predicted covariance matrix.

        Args:
            x: The 4-D current state vector of the vehicle.
            u: The 2-D input vector for the vehicle.

        Returns:
            The 4-D updated state of the vehicle based off of the motion model.
        """

        step_size = 1
        self.dt = self.dt/step_size
        for i in range(0,step_size):
            x = self.motion_model(x, u)

        if(x[2,0]>np.pi):
            x[2,0] = x[2,0]-2*np.pi
        if(x[2,0]<-np.pi):
            x[2,0] = x[2,0]+2*np.pi
        self.dt = self.dt*step_size
        F = self.calc_F(x[3][0], x[2][0], u[1][0])

        self.P = F @ self.P @ F.T + self.Q
        return x

    def correct(self, x, z):
        """The correction phase of the EKF.

        The observation correction phase for the EKF. The observation model is defined, and then the standard EKF motion model is applied, with angle regularization into the [-pi, pi] range applied intermitently.

        Args:
            x: The 4-D current state vector of the vehicle.
            z: The 3-D observation vector of the vehicle, consisting of cartesian coordinates x, y from the GPS measurement, and heading theta from the magnetometer.

        Returns:
            The corrected state of the vehicle based off of the sensor measurements.
        """
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ])
        zPred = H@x
        while(z[2,0]>np.pi):
            z[2,0] = z[2,0]-2*np.pi
        while(z[2,0]<-np.pi):
            z[2,0] = z[2,0]+2*np.pi

        y = z-zPred
        y[2,0] = self.angle_diff(z[2,0], zPred[2,0])
        if(y[2,0]>np.pi):
            y[2,0] = y[2,0]-2*np.pi
        if(y[2,0]<-np.pi):
            y[2,0] = y[2,0]+2*np.pi
        S = H@self.P@H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        x = x+K@y
        self.P = (np.eye(len(x))-K@H) @ self.P
        return x


