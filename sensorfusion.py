from filterpy.kalman import KalmanFilter
import numpy as np
import time
import matplotlib.pyplot as plt

class SensorFusion:
    def __init__(self):
        # Initialize Kalman filter for a 6D state: [x, y, theta, vx, vy, omega]
        self.kf = KalmanFilter(dim_x=6, dim_z=6)

        # State vector [x, y, theta, vx, vy, omega]
        self.kf.x = np.array([0, 0, 0, 0, 0, 0])


        self.kf.F = np.array([[1, 0, 0, 0, 0, 0],
                              [0, 1, 0, 0, 0, 0],
                              [0, 0, 1, 0, 0, 0],
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

        # Measurement noise covariance (R) for each measurement component (position)
        # We assume the camera and encoder have similar noise characteristics for position measurements
        self.kf.R = np.diag([1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1])  # Ignoring speed measurements

        # Process noise covariance (Q), reflecting model uncertainty
        self.kf.Q = np.eye(6) * 1e-1

        # Initial error covariance (P), reflects initial uncertainty in state estimates
        self.kf.P *= 0.1

    def get_estimated_position(self, encoder_position, encoder_speed, camera_position):
        # Prediction Step
        self.kf.predict()

        # Measurement vector (z) - combining both sensor positions
        z = np.array([camera_position[0], camera_position[1], camera_position[2],
                      encoder_position[0], encoder_position[1], encoder_position[2]])

        # Update Step with the combined measurement vector
        self.kf.update(z)

        # Extract estimated [x, y, theta] from the state vector
        estimated_position = self.kf.x[:3]  # [x, y, theta]

        # Return estimated position
        return estimated_position


def test_sensor_fusion():
    # Create a SensorFusion object
    sensor_fusion = SensorFusion()

    # Simulation parameters
    num_steps = 50
    time_step = 1  # seconds
    time_simulation = np.arange(0, num_steps * time_step, time_step)

    # Adjustable dropout duration
    dropout_duration = 1  # Number of steps for camera dropout

    # Generate constant speed and linearly increasing encoder positions
    encoder_speed = np.array([500, 3, 0.02])  # Constant speed [vx, vy, omega]
    encoder_position = np.array([0.0, 0.0, 0.0])
    true_position = np.array([0.0, 0.0, 0.0])
    # Arrays to store the simulation results
    true_positions = []
    estimated_positions = []
    camera_positions = []
    encoder_positions = []

    # Simulation loop
    for t in range(num_steps):
        true_position += encoder_speed * time_step
        true_positions.append(true_position.copy())
        camera_position = true_position * np.random.normal(1.0, 0.1, size=3)  # 10% error position from camera
        encoder_position = true_position * np.random.normal(1.0, 0.1, size=3)  # 10% error position from encoder
        estimated_position = sensor_fusion.get_estimated_position(
            encoder_position, encoder_speed, camera_position
        )

        estimated_positions.append(estimated_position)
        camera_positions.append(camera_position)
        encoder_positions.append(encoder_position)

    print(true_positions)
    true_positions = np.array(true_positions)
    estimated_positions = np.array(estimated_positions)
    camera_positions = np.array(camera_positions)
    encoder_positions = np.array(encoder_positions)

    # Calculate Mean Square Error (MSE) and normalize it
    mse = np.mean((true_positions - estimated_positions) ** 2, axis=0)
    ranges = np.max(true_positions, axis=0) - np.min(true_positions, axis=0)
    normalized_mse = mse / ranges  # Normalize MSE for each position component

    # Print normalized MSE values
    print("Normalized Mean Square Error for each position component:")
    for i, label in enumerate(["x", "y", "theta"]):
        print(f"{label} Position MSE: {normalized_mse[i]:.4f}")

    # Plot the results
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))
    labels = ["x", "y", "theta"]
    for i in range(3):
        axs[i].plot(time_simulation, true_positions[:, i], label="True Position", color='green')
        axs[i].plot(time_simulation, camera_positions[:, i], label="Camera Position", linestyle='--', color='blue')
        axs[i].plot(time_simulation, estimated_positions[:, i], label="Estimated Position", color='red')
        axs[i].plot(time_simulation, encoder_positions[:, i], label="Encoder Position", color='pink')
        axs[i].set_title(f"{labels[i]} Position Over Time")
        axs[i].set_xlabel("Time (s)")
        axs[i].set_ylabel(f"{labels[i]} (mm or radians)")
        axs[i].legend()

    plt.tight_layout()
    plt.savefig("result\\" + str(time.time()) + "_sensorFusion.png")
    plt.show()

if __name__ == "__main__":
    test_sensor_fusion()
    #main()