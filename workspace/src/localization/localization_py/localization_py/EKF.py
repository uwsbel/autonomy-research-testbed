import numpy as np
import math
import yaml


class EKF:
    ''' 
    A basic Extended Kalman Filter implementation. This currently uses the 4DOF motion model.
    
    Motion Model inputs: Vehicle control inputs (throttle, steering)

    Observation Model inputs: Sensor measurements (GPS, Magnetometer)
    '''

    def motion_model(self, x, u):
        ''' 
        The 4DOF dynamics model for the vehicle
        '''
        x[0,0] = x[0,0]+math.cos(x[2,0])*self.dt*x[3,0]
        x[1,0] = x[1,0]+math.sin(x[2,0])*self.dt*x[3,0]
        x[2,0] = x[2,0]+self.dt*x[3,0]*math.tan(u[1,0])/self.l
        f = self.tau_0*u[0,0]-self.tau_0*x[3,0]/(self.omega_0*self.r_wheel*self.gamma)

        x[3,0] = x[3,0]+ \
            self.dt*((self.r_wheel*self.gamma)/self.i_wheel)*(f-(x[3,0]*self.c_1)/(self.r_wheel*self.gamma)-self.c_0)

        return x

    def calc_F(self, v, theta, delta, throttle):
        '''
        The State Transition Matrix
        '''

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

    def __init__(self, dt):
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

    def angle_diff(self,ref, act):
        '''
        Modular subtraction for computing ange differences
        '''
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
        '''
        The prediction phase of the EKF
        '''
        step_size = 1
        self.dt = self.dt/step_size
        for i in range(0,step_size):
            x = self.motion_model(x, u)

        if(x[2,0]>np.pi):
            x[2,0] = x[2,0]-2*np.pi
        if(x[2,0]<-np.pi):
            x[2,0] = x[2,0]+2*np.pi
        self.dt = self.dt*step_size
        F = self.calc_F(x[3][0], x[2][0], u[1][0], u[0][0])

        self.P = F @ self.P @ F.T + self.Q
        return x

    def correct(self, x, z):
        '''
        The correction phase of the EKF
        '''
        #observation model
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


