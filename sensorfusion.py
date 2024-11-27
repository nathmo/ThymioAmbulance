import numpy as np
import time
import matplotlib.pyplot as plt


class ParticleFilter:
    def __init__(self, num_particles, range_min, range_max):
        self.num_particles = num_particles
        self.particles = np.random.uniform(range_min, range_max, size=(self.num_particles, 1))
        self.weights = np.ones(self.num_particles) / self.num_particles

    def predict(self, delta_t, speed):
        # Move particles based on speed or angular velocity
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

    def is_valid(self, value, min_val=1, max_val=10000):
        # assomption that we will never make more than 10m before the system update
        """Check if the sensor value is within a valid range."""
        return min_val <= value <= max_val

    def get_estimated_position(self, encoder_position, encoder_speed, camera_position, delta_t=1.0,
                               measurement_noise=1.0):
        if camera_position is None:
            camera_position = np.array([0, 0, 0])
        # Update x filter
        if self.is_valid(camera_position[0]) and self.is_valid(encoder_position[0]):
            # If both the camera and encoder values are valid, use the average
            updated_position = (camera_position[0] + encoder_position[0]) / 2
            self.x_filter.update(updated_position, measurement_noise)
        elif self.is_valid(camera_position[0]):
            # If only the camera value is valid, use it
            updated_position = camera_position[0]
            self.x_filter.update(updated_position, measurement_noise)
        elif self.is_valid(encoder_position[0]):
            # If only the encoder value is valid, use it
            updated_position = encoder_position[0]
            self.x_filter.update(updated_position, measurement_noise)
        else:
            None# at this point there is no miracle to be done..., dont update

        # Update the filter with the determined position
        self.x_filter.predict(delta_t, encoder_speed[0])  # Only consider speed along x


        # Update y filter
        if self.is_valid(camera_position[1]) and self.is_valid(encoder_position[1], min_val=0):
            updated_position = (camera_position[1] + encoder_position[1]) / 2
            self.y_filter.update(updated_position, measurement_noise)
        elif self.is_valid(camera_position[1]):
            updated_position = camera_position[1]
            self.y_filter.update(updated_position, measurement_noise)
        elif self.is_valid(encoder_position[1], min_val=0):
            updated_position = encoder_position[1]
            self.y_filter.update(updated_position, measurement_noise)
        else:
            None# at this point there is no miracle to be done..., dont update

        self.y_filter.predict(delta_t, encoder_speed[1])  # Only consider speed along y


        # Update theta filter
        if self.is_valid(camera_position[2], min_val=-np.pi, max_val=np.pi) and \
                self.is_valid(encoder_position[2], min_val=-np.pi, max_val=np.pi):
            updated_position = (camera_position[2] + encoder_position[2]) / 2
            self.theta_filter.update(updated_position, measurement_noise)
        elif self.is_valid(camera_position[2], min_val=-np.pi, max_val=np.pi):
            updated_position = camera_position[2]
            self.theta_filter.update(updated_position, measurement_noise)
        elif self.is_valid(encoder_position[2], min_val=-np.pi, max_val=np.pi):
            updated_position = encoder_position[2]
            self.theta_filter.update(updated_position, measurement_noise)
        else:
            None# at this point there is no miracle to be done..., dont update

        self.theta_filter.predict(delta_t, encoder_speed[2])  # Only consider angular speed (omega)


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

    # Time steps (e.g., from 0 to 49)
    t = np.arange(num_steps)

    # Offsets for each axis (you can customize them as needed)
    offset_x = 8
    offset_y = 2
    offset_theta = 5

    # Generate the speed for each axis
    speed_x = offset_x* np.sin(t/4+5) ** 2 + np.random.randn(num_steps) * 0.1
    speed_y = offset_y*np.sin(t/2-3) ** 2 + np.random.randn(num_steps) * 0.1
    speed_theta = offset_theta*np.sin(t/5+20)

    # Bundle speeds into a list (or any other format you prefer)
    true_speed = [np.array([speed_x[i], speed_y[i], speed_theta[i]]) for i in range(num_steps)]

    # Generate constant speed and linearly increasing encoder positions
    #true_speed = [np.array([50, 1, 0.1])]  # Constant speed [vx, vy, omega]
    print(true_speed)
    true_position = np.array([0.0, 0.0, 0.0])
    # Arrays to store the simulation results
    true_positions = []
    estimated_positions = []
    camera_positions = []
    encoder_positions = []

    # Simulation loop
    start_time = time.time()

    # Loop to simulate robot movement and sensor fusion
    for t in range(num_steps):
        loop_start_time = time.time()  # Start time of each iteration
        true_position += true_speed[t] * time_step
        true_positions.append(true_position.copy())
        camera_position = true_position * np.random.normal(1.0, 0.2, size=3)  # 10% error position from camera
        encoder_position = true_position * np.random.normal(1.0, 0.2, size=3)  # 10% error position from encoder
        encoder_speed = true_speed[t] * np.random.normal(1.0, 0.2, size=3)  # 10% error speed from encoder

        estimated_position = sensor_fusion.get_estimated_position(
            encoder_position, encoder_speed, camera_position
        )

        estimated_positions.append(estimated_position)
        camera_positions.append(camera_position)
        encoder_positions.append(encoder_position)

        # Calculate the time taken for this iteration and store it
        loop_end_time = time.time()  # End time of the current iteration
        loop_duration = loop_end_time - loop_start_time  # Time taken for this iteration

    # After the loop, calculate and print the average time per iteration
    end_time = time.time()
    total_duration = end_time - start_time
    average_time_per_iteration = total_duration / num_steps

    print(f"Total time for {num_steps} iterations: {total_duration:.2f} seconds")
    print(f"Average time per iteration: {average_time_per_iteration:.6f} seconds")
    # Average time per iteration: 0.004697 seconds on my small tablet...
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