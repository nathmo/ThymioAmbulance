from filterpy.kalman import KalmanFilter
import numpy as np


class SensorFusion:
    def __init__(self):
        # Initialize Kalman filter for a 6D state: [x, y, theta, vx, vy, omega]
        self.kf = KalmanFilter(dim_x=6, dim_z=6)

        # State vector [x, y, theta, vx, vy, omega]
        self.kf.x = np.array([0, 0, 0, 0, 0, 0])

        # State transition matrix (F) assuming constant velocity model
        dt = 1.0  # Time step, adjust as needed
        self.kf.F = np.array([[1, 0, 0, dt, 0, 0],
                              [0, 1, 0, 0, dt, 0],
                              [0, 0, 1, 0, 0, dt],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])

        # Measurement function (H) - now observing [x, y, theta, vx, vy, omega] directly
        self.kf.H = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0],
                              [0, 0, 0, 1, 0, 0],
                              [0, 0, 0, 0, 1, 0],
                              [0, 0, 0, 0, 0, 1]])

        # Measurement noise covariance (R) for each measurement component (position and velocity)
        self.kf.R = np.diag([1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2])

        # Process noise covariance (Q), reflecting model uncertainty
        self.kf.Q = np.eye(6) * 1e-4

        # Initial error covariance (P), reflects initial uncertainty in state estimates
        self.kf.P *= 1.0

    def get_estimated_position(self, encoder_position, encoder_speed, camera_position):
        # Prediction Step
        self.kf.predict()

        # Measurement vector (z) - combining camera position/orientation and encoder speeds
        z = np.array([camera_position[0], camera_position[1], camera_position[2],
                      encoder_speed[0], encoder_speed[1], encoder_speed[2]])

        # Update Step with the full measurement vector
        self.kf.update(z)

        # Extract estimated [x, y, theta, vx, vy, omega] from the state vector
        estimated_position = self.kf.x[:3]  # [x, y, theta]

        # Calculate the error between encoder position and estimated position
        error = encoder_position[:3] - estimated_position
        print("Error between encoder position and estimated position:", error)

        # Return estimated position and error
        return estimated_position
