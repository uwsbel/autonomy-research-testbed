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
        #new throttel model...
        # x[3,0] = x[3,0]+self.dt*(self.r_wheel*self.gamma/self.i_wheel)*(u[0,0]*self.new_f(x[3,0]) - (x[3,0]*self.c_1/(self.r_wheel*self.gamma)) - self.c_0)
        # if(x[3,0]<0):
        #     x[3,0] = 0
        # print(x[3,0])
        #if we try to use the new model the way the old one was used?
        #x[3,0] = x[3,0]*(1-self.dt*(self.c_1/self.i_wheel))+u[0,0]*self.dt*(self.tau_0/(self.i_wheel*self.omega_0))*(x[3,0]-self.gamma*self.r_wheel*self.omega_0)
        #print(1-self.dt*(self.c_1/self.i_wheel))
        #old model....
        #x[3,0] = x[3,0]*(1.0 + self.dt*((self.r_wheel*self.gamma/self.i_wheel)*(u[0, 0]*self.df_1_dv + (1-u[0, 0])*self.df_0_dv(x[3,0]))))+u[0,0]*(self.dt*((self.r_wheel*self.gamma/self.i_wheel)*(self.f_1_v(x[3, 0])-self.f_0_v(x[3, 0]))))
        
        #x[3,0] = x[3,0]+ \
        #        self.dt*((self.r_wheel*self.gamma/self.i_wheel)*(u[0,0]*self.new_f(x[3,0])-(x[3,0]*self.c_1/(self.r_wheel*self.gamma))-self.c_0))
        
        #if(x[3,0]<0):
        #    x[3,0]= 0
        
        ###THIS IS TEST...DEF NOT CORRECT EQ###
        throttle = u[0,0]
        max_vel = 2
        if(throttle<0.05):
            max_vel = throttle*20*0.2
        elif(throttle<0.1):
            max_vel = (throttle-0.05)*20*0.1+0.2
        elif(throttle<0.15):
            max_vel = (throttle-0.1)*20*0.1+0.3
        elif(throttle<0.2):
            max_vel = (throttle-0.15)*20*0.1+0.38
        elif(throttle<0.25):
            max_vel = (throttle-0.2)*20*0.1+0.5
        elif(throttle<0.3):
            max_vel = (throttle-0.25)*20*0.25+0.6
        elif(throttle<0.35):
            max_vel = (throttle-0.3)*20*0.2+0.85
        elif(throttle<0.4):
            max_vel = (throttle-0.35)*20*0.15+1.05
        elif(throttle<0.45):
            max_vel = (throttle-0.4)*20*0.1+1.2
        elif(throttle<0.5):
            max_vel = (throttle-0.45)*20*0.1+1.40
        elif(throttle<0.55):
            max_vel = (throttle-0.5)*20*0.1+1.40
        elif(throttle<0.6):
            max_vel = (throttle-0.55)*20*0.05+1.50
        elif(throttle<0.65):
            max_vel = (throttle-0.6)*20*0.05+1.55
        elif(throttle<0.7):
            max_vel = (throttle-0.65)*20*0.05+1.6
        elif(throttle<0.8):
            max_vel = (throttle-0.70)*10*0.5+1.65

        #print(max_vel)
        diff = max_vel-x[3,0]
        if(diff>0):
            accel = (diff**1.5)*throttle
        accel = (diff)*throttle
        x[3,0] = x[3,0]+accel

        ###END###
        # A = np.array([[1.0, 0, -self.dt*(x[3, 0]*math.sin(x[2, 0])), self.dt*(math.cos(x[2, 0]))],
        #               [0, 1.0, self.dt*(x[3, 0]*math.cos(x[2, 0])), self.dt*(math.sin(x[2, 0]))],
        #               [0, 0, 1.0, self.dt*(math.tan(u[1, 0])/self.l)],
        #               [0, 0, 0, 1.0 + self.dt*((self.r_wheel*self.gamma/self.i_wheel)*(u[0, 0]*self.df_1_dv + (1-u[0, 0])*self.df_0_dv(x[3,0])))]])
        # B = np.array([[0, 0],
        #               [0, 0],
        #               [0, 0],#self.dt*(x[3, 0]/(self.l*(math.cos(u[1, 0])**2)))],
        #               [self.dt*((self.r_wheel*self.gamma/self.i_wheel)*(self.f_1_v(x[3, 0])-self.f_0_v(x[3, 0])))/25, 0]])
        #x = A@x+B@u
        return x

    def new_f(self, v):
        ret = 0.08
        if(v!=0):
            self.omega_0 = (abs(v)*36)#/self.i_wheel
            if(v>0.5):
                ret = 1/(20*v)#-(self.tau_0*v/(self.omega_0*self.r_wheel*self.gamma))+self.tau_0
        #print(ret)
        if(ret<0):
            return 0
        return ret

    def f_1_v(self, v):
        return self.tau_0-v*self.tau_0/(self.omega_0*self.r_wheel*self.gamma)

    def f_0_v(self, v):
        if(v <= 0.1*self.omega_0*self.r_wheel):
            return 0
        else:
            return self.tau_0/9-10*self.tau_0*v/(9*self.omega_0*self.gamma*self.r_wheel)
    #Need to write this F

    def new_calc_F(self, v, theta, delta, throttle):
        #here, we need dx/dtheta, dx/dv, dy/dtheta, dy/dv. theta depends on both v and delta...
        F = np.array([
            [1.0, 0.0, -self.dt * v *
                math.sin(theta), self.dt * math.cos(theta)],
            [0.0, 1.0, self.dt * v *
                math.cos(theta), self.dt * math.sin(theta)],
            [0.0, 0.0, 1.0, self.dt*np.tan(delta)/self.l],
            [0.0, 0.0, 0.0, 1.0+(self.r_wheel*self.gamma/self.i_wheel)
             * (throttle*self.df_1_dv+(1-throttle)*self.df_0_dv(v))*self.dt]
        ])
        return F

    def df_0_dv(self, v):
        if(v <= 0.1*self.omega_0*self.r_wheel):
            return 0
        else:
            return -10*self.tau_0 / (9*self.omega_0*self.r_wheel*self.gamma)
    def __init__(self, dt):
        self.dt = dt
        #vehicle parameters:
        self.c_1 = 1e-4
        self.c_0 = 0.02
        self.l = 0.25
        self.r_wheel = 0.08451952624
        self.i_wheel = 1e-3
        self.gamma = 1/3
        self.tau_0 = 1.0
        self.omega_0 = 1300*8.0*np.pi/30
        self.df_1_dv = -self.tau_0/(self.omega_0*self.r_wheel*self.gamma)

        self.Q = np.diag([
            0.1,  # variance of location on x-axis
            0.1,  # variance of location on y-axis
            np.deg2rad(5),  # variance of yaw angle
            0.1 # variance of velocity
        ]) ** 2  # predict state covariance
        # Observation x,y position covariance
        self.R = np.diag([50.0, 50.0, 50.0]) ** 2
        self.P = np.eye(4)

    def predict(self, x, u):
        x = self.new_motion_model(x, u)
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