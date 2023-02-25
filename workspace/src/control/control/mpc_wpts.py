#
# BSD 3-Clause License
#
# Copyright (c) 2022 University of Wisconsin - Madison
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.#

#// =============================================================================
#// Author: Harry Zhang
#// =============================================================================

import osqp
import numpy as np
import scipy as sp
from scipy import sparse
import time

def mpc_wpts_solver(e,u,vel,vel_ref):
    
    x0 = np.array(e)
    u0 = np.array(u)

    # parameters related to the vehicle dynamics
    r_wheel = 0.08451952624
    i_wheel = 1e-3
    gamma = 1/3
    tau_0 = 0.09
    omega_0 = 161.185
    c1 = 1e-4
    l_car = 0.25
    nsim = 1
    delta_t = 0.1
    start_time = time.time()
    for i in range(nsim):

        #time dependent variables in Ad and Bd matrices
        alpha = u0[0]
        delta = u0[1]
        e1 = x0[0]
        e2 = x0[1]
        e3 = x0[2]
        #reference/target point
        xr = np.array([0, 0, 0.,0])

            # Discrete time model of a vehicle
        Ad = sparse.csc_matrix([
            [1.0,   vel*np.tan(delta)/l_car*delta_t,  -vel_ref*np.sin(e3)*delta_t,  0.],
            [-vel*np.tan(delta)/l_car*delta_t,   1.0,  vel_ref*np.cos(e3)*delta_t,  0.],
            [0.,   0.,  1.0,  0],
            [0.,   0.,  0.,  1.0-(c1/i_wheel)*delta_t],
        ])
        #Ad = Ad * delta_t + sparse.eye(4)

        Bd = sparse.csc_matrix([
                [0.,  vel*e2/(l_car*np.cos(delta)*np.cos(delta))*delta_t],
                [0.,  -vel*e1/(l_car*np.cos(delta)*np.cos(delta))*delta_t],
                [0.,  -vel/(l_car*np.cos(delta)*np.cos(delta))*delta_t],
                [tau_0*(vel_ref-gamma*r_wheel*omega_0)/(i_wheel*omega_0)*delta_t,  0]
                ])

        [nx, nu] = Bd.shape

        # Constraints
        #u0 = 10.5916
        umin = np.array([0.0,-0.6]) 
        umax = np.array([1, 0.6]) 
        xmin = np.array([-np.inf,-np.inf,-np.inf,-np.inf])
        xmax = np.array([np.inf, np.inf, np.inf,np.inf])
        # xmin = np.array([-0.75,-0.75,-np.inf,-np.inf])
        # xmax = np.array([0.75, 0.75, np.inf,np.inf])
        # Objective function
        Q = sparse.diags([5500., 5500., 500., 300.])
        QN = Q
        R = sparse.diags([10., 0.])



        # Prediction horizon
        N = 10

        # Cast MPC problem to a QP: x = (x(0),x(1),...,x(N),u(0),...,u(N-1))
        # - quadratic objective
        P = sparse.block_diag([sparse.kron(sparse.eye(N), Q), QN,
                            sparse.kron(sparse.eye(N), R)], format='csc')
        # - linear objective
        q = np.hstack([np.kron(np.ones(N), -Q.dot(xr)), -QN.dot(xr),
                    np.zeros(N*nu)])
        # - linear dynamics
        Ax = sparse.kron(sparse.eye(N+1),-sparse.eye(nx)) + sparse.kron(sparse.eye(N+1, k=-1), Ad)
        Bu = sparse.kron(sparse.vstack([sparse.csc_matrix((1, N)), sparse.eye(N)]), Bd)
        Aeq = sparse.hstack([Ax, Bu])
        leq = np.hstack([-x0, np.zeros(N*nx)])
        ueq = leq
        # - input and state constraints
        Aineq = sparse.eye((N+1)*nx + N*nu)
        lineq = np.hstack([np.kron(np.ones(N+1), xmin), np.kron(np.ones(N), umin)])
        uineq = np.hstack([np.kron(np.ones(N+1), xmax), np.kron(np.ones(N), umax)])
        # - OSQP constraints
        A = sparse.vstack([Aeq, Aineq], format='csc')
        l = np.hstack([leq, lineq])
        u = np.hstack([ueq, uineq])

        # Create an OSQP object
        prob = osqp.OSQP()

        # Setup workspace
        prob.setup(P, q, A, l, u, warm_start=True,max_iter=10000)

        # Simulate in closed loop


            # Solve
        res = prob.solve()

            # Check solver status
        if res.info.status != 'solved':
            raise ValueError('OSQP did not solve the problem!')

            # Apply first control input to the plant
        ctrl = res.x[-N*nu:-(N-1)*nu]
        u0 = ctrl
        x0 = Ad.dot(x0) + Bd.dot(ctrl)

            # Update initial state
        l[:nx] = -x0
        u[:nx] = -x0
        prob.update(l=l, u=u)
    
    print('For the error state: e=',e )
    print('Control input:',ctrl)
    
    #print('Next predict state',x0)
        #print(res.x)
    print("--- %s seconds ---" % (time.time() - start_time))
    return ctrl 