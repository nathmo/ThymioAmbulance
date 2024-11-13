import time
from robotmovement import RobotMovement
from visionsystem import VisionSystem
from sensorfusion import SensorFusion
from pathplanner import PathPlanner
from helper import *
import numpy as np
import matplotlib.pyplot as plt


def main():
    #
    robot = RobotMovement()
    vision = VisionSystem()
    sensorfusion = SensorFusion()

    # try using an image instead of the camera to speed up the test in the beginning.
    if(vision.is_camera_ready() and False):
        robotPosFromCamera = vision.get_robot_position()
        goalPosFromCamera = vision.get_goal_position()
        occupancyGrid = vision.generate_occupancy_grid()
    else:
        print("camera Not Ready") # we use static data instead to simulate the input from the camera.
        # Load the matrix and convert integers back to boolean
        matrix = np.loadtxt("testData\\mapWithBlackObstacle1.txt", dtype=int)
        # Convert the list of lists into a NumPy array
        occupancyGrid = matrix.astype(bool)
        robotPosFromCamera =np.array([1500, 2000, np.pi / 6]) # set a random value instead
        goalPosFromCamera =np.array([1000, 4500, np.pi / 6])
    print("robotPosFromCamera : " + str(robotPosFromCamera))
    print("goalPosFromCamera : " + str(vision.get_goal_position()))
    robotPosFromEncoder = np.array([1550,1950, np.pi / 6.05])# slightly different than the default camera one
    robotSpeedFromEncoder = np.array([0, 0, 0]) #no speed
    print("robotPosFromEncoder : " + str(robotPosFromEncoder))
    print("robotSpeedFromEncoder : " + str(robotSpeedFromEncoder))
    robotPosFromFusion = sensorfusion.get_estimated_position(robotPosFromEncoder, robotSpeedFromEncoder, robotPosFromCamera)
    print("robotPosFromFusion : " + str(robotPosFromFusion))
    waypoints = PathPlanner.get_waypoints(occupancyGrid, robotPosFromFusion, goalPosFromCamera)
    print("waypoints : " + str(waypoints))
    print("number of waypoints : " + str(len(waypoints)))

    plot_robot_grid(occupancyGrid, 10, robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, waypoints)
    test_sensor_fusion()

import numpy as np
import matplotlib.pyplot as plt
import time

def test_sensor_fusion():
    # Create a SensorFusion object
    sensor_fusion = SensorFusion()

    # Simulation parameters
    num_steps = 50
    time_step = 1  # seconds
    time_simulation = np.arange(0, num_steps * time_step, time_step)

    # Adjustable dropout duration
    dropout_duration = 20  # Number of steps for camera dropout

    # Generate constant speed and linearly increasing encoder positions
    encoder_speed = np.array([500, 3, 0.02])  # Constant speed [vx, vy, omega]
    encoder_position = np.array([0.0, 0.0, 0.0])

    # Arrays to store the simulation results
    true_positions = []
    estimated_positions = []
    camera_positions = []

    # Simulation loop
    for t in range(num_steps):
        encoder_position += encoder_speed * time_step
        true_position = encoder_position.copy()

        # Generate camera position with adjustable dropout duration
        if t % (dropout_duration + 3) < dropout_duration:
            camera_position = true_position * np.random.normal(1.0, 0.1, size=3) #10% error position from camera
        else:
            camera_position = camera_positions[-1] if camera_positions else true_position.copy()

        estimated_position = sensor_fusion.get_estimated_position(
            true_position, encoder_speed, camera_position
        )

        true_positions.append(true_position)
        estimated_positions.append(estimated_position)
        camera_positions.append(camera_position)

        print(f"Step {t}: True Position = {true_position}")

    true_positions = np.array(true_positions)
    estimated_positions = np.array(estimated_positions)
    camera_positions = np.array(camera_positions)

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
        axs[i].set_title(f"{labels[i]} Position Over Time")
        axs[i].set_xlabel("Time (s)")
        axs[i].set_ylabel(f"{labels[i]} (mm or radians)")
        axs[i].legend()

    plt.tight_layout()
    plt.savefig("result\\" + str(time.time()) + "_sensorFusion.png")
    plt.show()

# Run the test function
test_sensor_fusion()



if __name__ == "__main__":
    test_sensor_fusion()
    #main()
