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


if __name__ == "__main__":
    main()
