import numpy as np
import math
import yaml


class EKF:
    """A basic Extended Kalman Filter implementation.

    This currently uses the 4DOF motion model. The Motion Model inputs are throttle and steering, and the observation model uses GPS and Magnetometer as sensors. States are cartesian coordinates x, y, heading theta, and velocity v.

    Attributes:
        Q: The covariane matrix for our states.
        R: The covariance matrix for our sensors.
        P: The predicted covariance estimate.
        dt: The timestep for the filter.
    """

    def __init__(self, dt, dynamics, Q, R):
        """Initialize the EKF.

        Initialize each of the global variables for the EKF. Load the dynamics parameters for the vehicle from an external YAML file.

        Args:
            dt: The timestep at which this filter will operate at.
            dynamics_path: The path to the dynamics parameters yaml file.
            param_path: The path to the filter parameters yaml file
        """

        self.dt = dt
        self.dyn = dynamics

        self.Q = (
            np.diag(
                [
                    Q[0],  # variance of location on x-axis
                    Q[0],  # variance of location on y-axis
                    Q[2],  # variance of yaw angle
                    Q[3],  # variance of velocity
                ]
            )
            ** 2
        )  # predict state covariance
        # Observation x,y position covariance
        self.R = np.diag([R[0], R[0], R[1]]) ** 2
        self.P = np.eye(4)

    def angle_diff(self, ref, act):
        """Computes the difference between two angles.

        Computes the difference between two angles in radians using modular subtraction. This solves the issue when we have a positive and negative angle, both with very large magnitude.

        Args:
            ref: The reference angle in radians.
            act: the actual angle in radians.

        Returns:
            The difference between the two angles, using modular arithmatic. For example, an input of {pi, pi/2} will return pi/2. An input of {-pi+epsilon, pi-epsilon} will return 2 epsilon.
        """

        if (ref > 0 and act > 0) or (ref <= 0 and act <= 0):
            err_theta = ref - act
        elif ref <= 0 and act > 0:
            if abs(ref - act) < abs(2 * np.pi + ref - act):
                err_theta = -abs(act - ref)
            else:
                err_theta = abs(2 * np.pi + ref - act)
        else:
            if abs(ref - act) < abs(2 * np.pi - ref + act):
                err_theta = abs(act - ref)
            else:
                err_theta = -abs(2 * np.pi - ref + act)
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
        self.dt = self.dt / step_size
        for i in range(0, step_size):
            x = self.dyn.motion_model(x, u)

        if x[2, 0] > np.pi:
            x[2, 0] = x[2, 0] - 2 * np.pi
        if x[2, 0] < -np.pi:
            x[2, 0] = x[2, 0] + 2 * np.pi
        self.dt = self.dt * step_size
        F = self.dyn.calc_F(x[3][0], x[2][0], u[1][0])

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
        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]])
        zPred = H @ x
        while z[2, 0] > np.pi:
            z[2, 0] = z[2, 0] - 2 * np.pi
        while z[2, 0] < -np.pi:
            z[2, 0] = z[2, 0] + 2 * np.pi

        y = z - zPred
        y[2, 0] = self.angle_diff(z[2, 0], zPred[2, 0])
        if y[2, 0] > np.pi:
            y[2, 0] = y[2, 0] - 2 * np.pi
        if y[2, 0] < -np.pi:
            y[2, 0] = y[2, 0] + 2 * np.pi
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        x = x + K @ y
        self.P = (np.eye(len(x)) - K @ H) @ self.P
        return x
