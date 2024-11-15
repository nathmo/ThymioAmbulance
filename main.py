import threading
import time
from robotmovement import RobotMovement
from visionsystem import VisionSystem
from sensorfusion import SensorFusion
from pathplanner import PathPlanner
import numpy as np

def update_loop(robot):
    """Function to run robot.update() in a separate thread at ~100 Hz."""
    interval = 0.01  # 10 ms interval (100 Hz)

    while True:
        start_time = time.perf_counter()  # Record the start time

        # Call the update function
        robot.update()

        # Calculate the elapsed time and sleep for the remainder of the interval
        elapsed_time = time.perf_counter() - start_time
        sleep_time = max(0, interval - elapsed_time)  # Ensure non-negative sleep time
        if(sleep_time == 0):
            print("Fast loop cant keep up. please lower frequency or optimize better")
        time.sleep(sleep_time)

def main():
    #
    robot = RobotMovement()
    vision = VisionSystem()
    sensorfusion = SensorFusion()
    # Start a separate thread for the update loop
    update_thread = threading.Thread(target=update_loop, args=(robot,))
    update_thread.daemon = True  # Daemonize thread to exit when main program exits
    update_thread.start()

    while True:
        '''
        - get the Camera position of the map and robot and the target (car crash, ciff)
        - get the map layout (road line) (Camera)
        - get the position from the robot encoder
        - use sensorfusion to improve robot position
        - update robot position
        - use path planning to find best way (A*)
        - update robot movement instruction
        '''
        if(vision.is_camera_ready()):
            robotPosFromCamera = vision.get_robot_position()
            goalPosFromCamera = vision.get_goal_position()
            occupancyGrid = vision.generate_occupancy_grid()
            robotPosFromEncoder = robot.get_position()
            robotSpeedFromEncoder = robot.get_speed()
            robotPosFromFusion = sensorfusion.get_estimated_position(robotPosFromEncoder, robotSpeedFromEncoder, robotPosFromCamera)
            waypoints = PathPlanner.get_waypoints(occupancyGrid, robotPosFromFusion, goalPosFromCamera)
            robot.set_position(robotPosFromFusion)
            robot.set_waypoints(waypoints)
            print("Main ran and have the following intermediate value : ")
            print("waypoints : " + str(waypoints))
            print("number of waypoints : " + str(len(waypoints)))
            print("robotPosFromCamera : "+str(robotPosFromCamera))
            print(" + robotPosFromEncoder : " + str(robotPosFromEncoder))
            print(" = robotPosFromFusion : " + str(robotPosFromFusion))
            print("robotSpeedFromEncoder : " + str(robotSpeedFromEncoder))
            print("goalPosFromCamera : " + str(goalPosFromCamera))
            print("---------------------------------------------------------")
        else:
            print("camera Not Ready")


        time.sleep(1)  # Adjust the main loop's frequency as needed

if __name__ == "__main__":
    main()
