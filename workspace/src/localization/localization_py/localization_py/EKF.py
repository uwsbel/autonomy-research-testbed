import numpy as np
import math


class EKF(object):
    #Want to replace this motion model with Harry's from MPC solver...


    #as per Harry's computations...
    def new_motion_model(self, x, u):

        x[0,0] = x[0,0]+math.cos(x[2,0])*self.dt*x[3,0]
        x[1,0] = x[1,0]+math.sin(x[2,0])*self.dt*x[3,0]
        x[2,0] = x[2,0]+self.dt*x[3,0]*math.tan(u[1,0])/self.l
        f = self.tau_0*u[0,0]-self.tau_0*x[3,0]/(self.omega_0*self.r_wheel*self.gamma)

        x[3,0] = x[3,0]+ \
            self.dt*((self.r_wheel*self.gamma)/self.i_wheel)*(f-(x[3,0]*self.c_1)/(self.r_wheel*self.gamma)-self.c_0)

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

    def df_0_dv(self, v):
        if(v <= 0.1*self.omega_0*self.r_wheel):
            return 0
        else:
            return -10*self.tau_0 / (9*self.omega_0*self.r_wheel*self.gamma)
    def __init__(self, dt):#, q1, q3, q4, r1, r3):
        self.dt = dt
        #vehicle parameters:
        self.c_1 = 1e-4
        self.c_0 = 0.02#0.039
        self.l = 0.5
        self.r_wheel = 0.08451952624
        self.i_wheel = 1e-3
        self.gamma = 1/3
        self.tau_0 = 0.3#0.09
        self.omega_0 = 30#161.185
        self.df_1_dv = -self.tau_0/(self.omega_0*self.r_wheel*self.gamma)


        # self.Q = np.diag([
        #     q1,  # variance of location on x-axis
        #     11,  # variance of location on y-axis
        #     np.deg2rad(q3),  # variance of yaw angle
        #     q4 # variance of velocity
        # ]) ** 2  # predict state covariance
        # # Observation x,y position covariance
       # self.R = npediag([r1, r1, r3]) ** 2
        # self.P = np.eye(4)
        self.Q = np.diag([
            0.1,  # variance of location on x-axis
            0.1,  # variance of location on y-axis
            np.deg2rad(3),  # variance of yaw angle
            0.1 # variance of velocity
        ]) ** 2  # predict state covariance
        # Observation x,y position covariance
        self.R = np.diag([0.0, 0.0, 0.3]) ** 2
        self.P = np.eye(4)




        #BEST SO FAR:
        # self.Q = np.diag([
        #     0.2,  # variance of location on x-axis
        #     0.2,  # variance of location on y-axis
        #     np.deg2rad(5),  # variance of yaw angle
        #     0.1 # variance of velocity
        # ]) ** 2  # predict state covariance
        # # Observation x,y position covariance
        # self.R = np.diag([2.0, 2.0, 1.0]) ** 2
        # self.P = np.eye(4)


    def angle_diff(self,ref, act):
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
        step_size = 1
        self.dt = self.dt/step_size
        for i in range(0,step_size):
            x = self.new_motion_model(x, u)

        if(x[2,0]>np.pi):
            x[2,0] = x[2,0]-2*np.pi
        if(x[2,0]<-np.pi):
            x[2,0] = x[2,0]+2*np.pi
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
        #print("z is: "+ str(z))
        #print("and zpred is: " + str(zPred))\
        while(z[2,0]>np.pi):
            z[2,0] = z[2,0]-2*np.pi
        while(z[2,0]<-np.pi):
            z[2,0] = z[2,0]+2*np.pi

        y = z-zPred
        y[2,0] = self.angle_diff(z[2,0], zPred[2,0])
        #print("and y is: "+ str(y))
        if(y[2,0]>np.pi):
            y[2,0] = y[2,0]-2*np.pi
        if(y[2,0]<-np.pi):
            y[2,0] = y[2,0]+2*np.pi
        S = H@self.P@H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        #print("K@Y is: "+ str(K@y))
        x = x+K@y
        # while(x[2,0]>np.pi):
        #     x[2,0] = x[2,0]-2*np.pi
        # while(x[2,0]<-np.pi):
        #     x[2,0] = x[2,0]+2*np.pi
        self.P = (np.eye(len(x))-K@H) @ self.P
        return x


