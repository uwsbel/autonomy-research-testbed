import numpy as np
import yaml
import math
class Dynamics:
    """A 4DOF dynamics implementation.
    
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
    """
    def __init__(self, dt, path):
        """Initialize the dynamics object.

        Initialize each of the parameters for the 4DOF dynamics model. Load the dynamics parameters for the vehicle from an external YAML file.

        Args:
            dt: The timestep at which this filter will operate at. 
            path: The path to the dynamics parameters yaml file.
        """

        with open(path, 'r') as yaml_file:
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

        This is a linearization of the above motion model, used for updating the predicted covariance matrix in the EKF. This funciton returns the state transition matrix based off of the states and inputs that appear in partial derivatives.

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