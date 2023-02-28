from cvxpy import *
import numpy as np
import scipy as sp
from scipy import sparse
import time 




def mpc_cvxpy_solver_v2(xref,yref,v_current,u_current):

    start_time = time.time()
    x_0 = 0.0
    y_0 = 0.0
    theta0 = 0.0
    v = v_current
    x0 = np.array([x_0,y_0,theta0,v])
    u0 = np.array(u_current)

    # parameters related to the vehicle dynamics
    r_wheel = 0.08451952624
    i_wheel = 1e-3
    gamma = 1/3
    tau_0 = 1.0
    omega_0 = 1300*8.0*np.pi/30
    l_car = 0.5
    df_1_dv = -tau_0/(omega_0*r_wheel*gamma)
    df_0_dv = -10*tau_0 / (9*omega_0*r_wheel*gamma)
    nsim = 2
    delta_t = 0.1
    for i in range(nsim):
        #time dependent variables in Ad and Bd matrices
        alpha = u0[0]
        delta = u0[1]
        theta0 = x0[2]
        v = x0[3]
        f_0_v = -10*tau_0*v/(9*omega_0*r_wheel*gamma)+tau_0/9
        f_1_v = -tau_0*v/(omega_0*r_wheel*gamma)+tau_0
        #reference/target point
        xr = np.array([xref, yref, 0.,0.])

        if v < (0.1*omega_0*r_wheel):
            # Discrete time model of a vehicle
            Ad = sparse.csc_matrix([
                [1.0,   0.,  -v*np.sin(theta0)*delta_t,  np.cos(theta0)*delta_t],
                [0.,   1.0,  v*np.cos(theta0)*delta_t,  np.sin(theta0)*delta_t],
                [0.,   0.,  1.0,  np.tan(delta)/l_car*delta_t],
                [0.,   0.,  0.,  1.0+r_wheel*gamma/i_wheel*(alpha*df_1_dv)*delta_t],
            ])
            #Ad = Ad * delta_t + sparse.eye(4)

            Bd = sparse.csc_matrix([
                    [0.,  0.],
                    [0.,  0.],
                    [0.,  v/(l_car*np.cos(delta)*np.cos(delta))*delta_t],
                    [r_wheel*gamma/i_wheel*f_1_v*delta_t,  0]
                    ])
            #Bd = Bd * delta_t
        if v > (0.1*omega_0*r_wheel):
            Ad = sparse.csc_matrix([
                [1.0,   0.,  -v*np.sin(theta0)*delta_t,  np.cos(theta0)*delta_t],
                [0.,   1.0,  v*np.cos(theta0)*delta_t,  np.sin(theta0)*delta_t],
                [0.,   0.,  1.0,  np.tan(delta)/l_car*delta_t],
                [0.,   0.,  0.,  1.0+r_wheel*gamma/i_wheel*(alpha*df_1_dv+(1-alpha)*df_0_dv)*delta_t],
            ])
            #Ad = Ad * delta_t + sparse.eye(4)
            Bd = sparse.csc_matrix([
                    [0.,  0.],
                    [0.,  0.],
                    [0.,  v/(l_car*np.cos(delta)*np.cos(delta))*delta_t],
                    [r_wheel*gamma/i_wheel*(f_1_v-f_0_v)*delta_t,  0]
                    ])
            #Bd = Bd * delta_t

        [nx, nu] = Bd.shape
        # Constraints
        umin = np.array([0,-0.6]) 
        umax = np.array([1, 0.6]) 
        xmin = np.array([-np.inf,-np.inf,-np.inf,0])
        xmax = np.array([np.inf, np.inf, np.inf,omega_0*r_wheel])

        # Objective function
        Q = sparse.diags([2000., 2000., 0., 100.])
        QN = Q
        R = sparse.eye(2)



        # Prediction horizon
        N = 10

        # Define problem
        u = Variable((nu, N))
        x = Variable((nx, N+1))
        x_init = Parameter(nx)
        objective = 0
        constraints = [x[:,0] == x_init]
        for k in range(N):
            objective += quad_form(x[:,k] - xr, Q) + quad_form(u[:,k], R)
            constraints += [x[:,k+1] == Ad*x[:,k] + Bd*u[:,k]]
            constraints += [xmin <= x[:,k], x[:,k] <= xmax]
            constraints += [umin <= u[:,k], u[:,k] <= umax]
        objective += quad_form(x[:,N] - xr, QN)
        prob = Problem(Minimize(objective), constraints)

        # Simulate in closed loop

        x_init.value = x0
        prob.solve(solver=OSQP, warm_start=True)
        x0 = Ad.dot(x0) + Bd.dot(u[:,0].value)
        u0 = u[:,0].value
        print(u0)
        #print(u.value)
        #print(Ad)
        print("--- %s seconds ---" % (time.time() - start_time))

    return u0