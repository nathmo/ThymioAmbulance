import time
from robotmovement import RobotMovement
from visionsystem import VisionSystem
from sensorfusion import SensorFusion
from pathplanner import PathPlanner
import numpy as np
import matplotlib.pyplot as plt

def plot_robot_grid(occupancyGrid, robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, waypoints):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Display the occupancy grid as a black and white grid
    grid_size = 10  # each cell is 10mm
    height, width = occupancyGrid.shape
    ax.imshow(occupancyGrid, cmap='gray', origin='lower', extent=[0, width * grid_size, 0, height * grid_size])

    # Plot robot and goal positions with their orientations
    def plot_position_with_orientation(position, color, label):
        x, y, theta = position
        ax.plot(x, y, 'o', color=color, label=label)
        ax.arrow(x, y, np.cos(theta) * 10, np.sin(theta) * 10, head_width=5, head_length=5, fc=color, ec=color)

    plot_position_with_orientation(robotPosFromEncoder, 'blue', 'Robot Encoder Position')
    plot_position_with_orientation(robotPosFromCamera, 'green', 'Robot Camera Position')
    plot_position_with_orientation(robotPosFromFusion, 'purple', 'Robot Fusion Position')
    plot_position_with_orientation(goalPosFromCamera, 'red', 'Goal Position')

    # Plot waypoints
    for idx, waypoint in enumerate(waypoints):
        x, y, theta = waypoint
        ax.plot(x, y, 'x', color='orange')
        ax.annotate(f'WP{idx}', (x, y), textcoords="offset points", xytext=(5, 5), ha='center', color='orange')

    # Set plot limits and labels
    ax.set_xlim([0, width * grid_size])
    ax.set_ylim([0, height * grid_size])
    ax.set_xlabel("X position (mm)")
    ax.set_ylabel("Y position (mm)")
    ax.set_title("Occupancy Grid and Robot Positions")
    ax.legend(loc='upper right')

    plt.show()

# Sample data from user's code
occupancyGrid = np.array([[0, 1, 0], [0, 1, 0], [0, 0, 0]], dtype=bool)  # simple 3x3 test grid
robotPosFromEncoder = np.array([160, 202, np.pi / 6.05])
robotPosFromCamera = np.array([150, 200, np.pi / 6])
robotPosFromFusion = np.array([155, 201, np.pi / 6.02])
goalPosFromCamera = np.array([100, 150, np.pi / 6])
waypoints = [np.array([120, 180, np.pi / 4]), np.array([130, 190, np.pi / 3]), np.array([140, 200, np.pi / 2])]

# Call the function to display the plot
plot_robot_grid(occupancyGrid, robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, waypoints)

def main():
    #
    robot = RobotMovement()
    vision = VisionSystem()
    sensorfusion = SensorFusion()

    # try using an image instead of the camera to speed up the test in the beginning.
    if(vision.is_camera_ready()):
        robotPosFromCamera = vision.get_robot_position()
        goalPosFromCamera = vision.get_goal_position()
        occupancyGrid = vision.generate_occupancy_grid()
    else:
        print("camera Not Ready")

        # Load the matrix and convert integers back to boolean
        matrix = np.loadtxt("testData\mapWithBlackObstacle1.txt", dtype=int)
        # Convert the list of lists into a NumPy array
        occupancyGrid = np.array(matrix.astype(bool))
        robotPosFromCamera =np.array([150, 200, np.pi / 6]) # set a random value instead
        goalPosFromCamera =np.array([100, 150, np.pi / 6])
    print("robotPosFromCamera : " + str(robotPosFromCamera))
    print("goalPosFromCamera : " + str(vision.get_goal_position()))
    robotPosFromEncoder = np.array([160, 202, np.pi / 6.05])# slightly different than the default camera one
    robotSpeedFromEncoder = np.array([0, 0, 0]) #no speed
    print("robotPosFromEncoder : " + str(robotPosFromEncoder))
    print("robotSpeedFromEncoder : " + str(robotSpeedFromEncoder))
    robotPosFromFusion = sensorfusion.get_estimated_position(robotPosFromEncoder, robotSpeedFromEncoder, robotPosFromCamera)
    print("robotPosFromFusion : " + str(robotPosFromFusion))
    waypoints = PathPlanner.get_waypoints(occupancyGrid, robotPosFromFusion, goalPosFromCamera)
    print("waypoints : " + str(waypoints))
    print("number of waypoints : " + str(len(waypoints)))

    plot_robot_grid(occupancyGrid, robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, waypoints)


if __name__ == "__main__":
    main()
