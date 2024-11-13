from filterpy.kalman import KalmanFilter
import numpy as np
import time
import matplotlib.pyplot as plt


class ParticleFilter:
    def __init__(self, num_particles, range_min, range_max):
        self.num_particles = num_particles
        self.particles = np.random.uniform(range_min, range_max, size=(self.num_particles, 1))
        self.weights = np.ones(self.num_particles) / self.num_particles

    def predict(self, delta_t, speed, angular_velocity):
        # Move particles based on speed and angular velocity
        noise = np.random.randn(self.num_particles) * 0.1  # Adding noise to the prediction
        self.particles[:, 0] += speed * delta_t + noise

    def update(self, sensor_position, measurement_noise):
        # Update particle weights based on sensor measurements
        dist = np.abs(self.particles[:, 0] - sensor_position)
        likelihood = np.exp(-0.5 * (dist ** 2 / measurement_noise ** 2))
        self.weights = likelihood

        # Normalize the weights so that they sum to 1
        weight_sum = np.sum(self.weights)
        if weight_sum < 1e-10:
            self.weights = np.ones(self.num_particles) / self.num_particles
        else:
            self.weights /= weight_sum

    def resample(self):
        # Resample particles based on updated weights
        indices = np.random.choice(range(self.num_particles), size=self.num_particles, p=self.weights)
        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles  # Reset weights

    def estimate(self):
        # Estimate position as the mean of the particles
        return np.mean(self.particles)


class SensorFusion:
    def __init__(self, num_particles=10000, x_range=(0, 1000), y_range=(0, 1000), theta_range=(-np.pi, np.pi)):
        # Number of particles for each filter
        self.num_particles = num_particles

        # Initialize three particle filters (one for x, one for y, one for theta)
        self.x_filter = ParticleFilter(self.num_particles, [x_range[0]], [x_range[1]])
        self.y_filter = ParticleFilter(self.num_particles, [y_range[0]], [y_range[1]])
        self.theta_filter = ParticleFilter(self.num_particles, [theta_range[0]], [theta_range[1]])

    def get_estimated_position(self, encoder_position, encoder_speed, camera_position, delta_t=1.0,
                               measurement_noise=1.0):
        # Update each particle filter with the corresponding sensor data
        # 1. Update x filter
        self.x_filter.predict(delta_t, encoder_speed[0], 0)  # Only consider speed along x
        self.x_filter.update(camera_position[0], measurement_noise)

        # 2. Update y filter
        self.y_filter.predict(delta_t, encoder_speed[1], 0)  # Only consider speed along y
        self.y_filter.update(camera_position[1], measurement_noise)

        # 3. Update theta filter
        self.theta_filter.predict(delta_t, 0, encoder_speed[2])  # Only consider angular speed (omega)
        self.theta_filter.update(camera_position[2], measurement_noise)

        # Resample each filter's particles
        self.x_filter.resample()
        self.y_filter.resample()
        self.theta_filter.resample()

        # Estimate the final position as the weighted mean of each filter's particles
        estimated_x = self.x_filter.estimate()
        estimated_y = self.y_filter.estimate()
        estimated_theta = self.theta_filter.estimate()

        return np.array([estimated_x, estimated_y, estimated_theta])


def test_sensor_fusion():
    # Create a SensorFusion object
    sensor_fusion = SensorFusion()

    # Simulation parameters
    num_steps = 50
    time_step = 1  # seconds
    time_simulation = np.arange(0, num_steps * time_step, time_step)

    # Generate constant speed and linearly increasing encoder positions
    true_speed = np.array([50, 1, 0.1])  # Constant speed [vx, vy, omega]
    true_position = np.array([0.0, 0.0, 0.0])
    # Arrays to store the simulation results
    true_positions = []
    estimated_positions = []
    camera_positions = []
    encoder_positions = []

    # Simulation loop
    for t in range(num_steps):
        true_position += true_speed * time_step
        true_positions.append(true_position.copy())
        camera_position = true_position * np.random.normal(1.0, 0.1, size=3)  # 10% error position from camera
        encoder_position = true_position * np.random.normal(1.0, 0.1, size=3)  # 10% error position from encoder
        encoder_speed = true_speed * np.random.normal(1.0, 0.1, size=3)  # 10% error position from encoder
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