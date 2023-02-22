import numpy as np
import math


class EKF(object):
    #Want to replace this motion model with Harry's from MPC solver...


    #as per Harry's computations...
    def new_motion_model(self, x, u):
        #x[0,0] = x
        #x[1,0] = y
        #x[2,0] = heading theta (yaw)
        #x[3,0] = vel
        #u[0,0] = throttle alpha
        #u[1,0] = steering delta
        #A' and B'
        # if(not(u[0,0] == 0)):
        #     alpha = (14*math.e**(1/math.sqrt(u[0,0])))/math.e
        # else:
        #     alpha = 1
        #alpha = 50
        x[0,0] = x[0,0]+math.cos(x[2,0])*self.dt*x[3,0]
        x[1,0] = x[1,0]+math.sin(x[2,0])*self.dt*x[3,0]
        x[2,0] = x[2,0]+self.dt*x[3,0]*math.tan(u[1,0])/self.l
       
        f = self.tau_0*u[0,0]-self.tau_0*x[3,0]/(self.omega_0*self.r_wheel*self.gamma)

        x[3,0] = x[3,0]+ \
            self.dt*((self.r_wheel*self.gamma)/self.i_wheel)*(f-(x[3,0]*self.c_1)/(self.r_wheel*self.gamma)-self.c_0)

        if(x[2,0]<-math.pi):
            x[2,0] = x[2,0]+2*math.pi
        if(x[2,0]>math.pi):
            x[2,0] = x[2,0]-2*math.pi

        if(x[3,0]<0):
            x[3,0]= 0        
        return x


    def new_calc_F(self, v, theta, delta, throttle):
        #here, we need dx/dtheta, dx/dv, dy/dtheta, dy/dv. theta depends on both v and delta...
        #TODO: NEED TO RECOMPUTE THESE DERIVATIVES
        F = np.array([
            [1.0, 0.0, -self.dt * v *
                math.sin(theta), self.dt * math.cos(theta)],
            [0.0, 1.0, self.dt * v *
                math.cos(theta), self.dt * math.sin(theta)],
            [0.0, 0.0, 1.0, self.dt*np.tan(delta)/self.l],
            [0.0, 0.0, 0.0, 
            (self.dt*self.r_wheel*self.gamma/(self.i_wheel))*(-(self.tau_0/(self.omega_0*self.r_wheel*self.gamma))-(self.c_1/(self.r_wheel*self.gamma)))]
            
            
            #1.0+(self.r_wheel*self.gamma/self.i_wheel)
            # * (throttle*self.df_1_dv+(1-throttle)*self.df_0_dv(v))*self.dt]
        ])
        return F


    def __init__(self, dt):
        self.dt = dt
        #vehicle parameters:
        self.c_1 = 1e-4
        self.c_0 = 0.02
        self.l = 0.25
        self.r_wheel = 0.08451952624
        self.i_wheel = 1e-3
        self.gamma = 1/3
        self.tau_0 = 0.09#1
        self.omega_0 = 0.16*1300*7.4*0.1047198#1300*8.0*np.pi/30
        self.df_1_dv = -self.tau_0/(self.omega_0*self.r_wheel*self.gamma)

        self.Q = np.diag([
            0.3,  # variance of location on x-axis
            0.3,  # variance of location on y-axis
            np.deg2rad(5),  # variance of yaw angle
            0.3 # variance of velocity
        ]) ** 2  # predict state covariance
        # Observation x,y position covariance
        self.R = np.diag([4.0, 4.0, 40.0]) ** 2
        self.P = np.eye(4)

    def predict(self, x, u):
        step_size = 10
        self.dt = self.dt/step_size
        for i in range(0,step_size):
            x = self.new_motion_model(x, u)
        self.dt = self.dt*step_size
        F = self.new_calc_F(x[3][0], x[2][0], u[1][0], u[0][0])

        self.P = F @ self.P @ F.T + self.Q
        return x

    def correct(self, x, z):
        #observation model
        H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, 0]
        ])
        zPred = H@x
        y = z-zPred
        S = H@self.P@H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        x = x+K@y
        self.P = (np.eye(len(x))-K@H) @ self.P
        return x