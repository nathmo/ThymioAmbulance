import threading
import time
from robotmovement import RobotMovement
from visionsystem import VisionSystem
from sensorfusion import SensorFusion
from pathplanner import PathPlanner
import numpy as np
from visualisation import Visualisation
import os

def update_loop(robot):
    """Function to run robot.update() in a separate thread at ~1 Hz."""
    interval = 1  # 1000 ms interval (1 Hz)

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
    robot = RobotMovement()#debug=False) # debug=True -> dont need robot for simulation
    robot.connect()
    vision = VisionSystem(use_camera=True, cameraID=1, image_path=os.path.join("testData", "test.jpg"))
    sensorfusion = SensorFusion()
    pathplanner = PathPlanner(pixel_size_mm=vision.get_pixel_side_mm())
    # Start a separate thread for the update loop
    #update_thread = threading.Thread(target=update_loop, args=(robot,))
    #update_thread.daemon = True  # Daemonize thread to exit when main program exits
    #update_thread.start()
    visualizer = Visualisation(vision.get_pixel_side_mm())
    fameA = vision.generate_occupancy_grid()
    fameB = vision.get_frame()
    #visualizer.update_background(fameA, fameB)
    interval = 5  # 5000 ms interval (0.2 Hz)

    robotPosFromCamera = vision.get_robot_position()
    goalPosFromCamera = vision.get_goal_position()
    occupancyGrid = vision.generate_occupancy_grid()
    waypoints = pathplanner.get_waypoints(occupancyGrid, robotPosFromCamera, goalPosFromCamera)
    #robot.set_waypoints(waypoints)
    #robot.set_position(robotPosFromCamera)

    # compute only once the waypoints.
    while True:
        start_time = time.perf_counter()  # Record the start time
        '''
        - get the Camera position of the map and robot and the target 
        - get the map layout (road line) (Camera)
        - get the position from the robot encoder
        - use sensorfusion to improve robot position
        - update robot position
        - use path planning to find best way (A*)
        - update robot movement instruction
        '''

        # add a line to set pixel scale to the path finder
        robotPosFromCamera = vision.get_robot_position()
        goalPosFromCamera = vision.get_goal_position()
        robotPosFromEncoder = vision.get_robot_position()
        robotSpeedFromEncoder = np.array([0.0, 0.0, 0.0, 0.0])
        elapsed_time = time.perf_counter() - start_time
        # robot.set_position(robotPosFromEncoder)
        robotPosFromFusion = sensorfusion.get_estimated_position(robotPosFromEncoder, robotSpeedFromEncoder, robotPosFromCamera, delta_t=elapsed_time)
        robot.set_position(robotPosFromFusion)
        robot.update()
        # Update plot dynamically
        fameB = vision.get_frame()
        visualizer.update_plot(fameA, fameB, robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, waypoints)
        #visualizer.update_background(fameA, fameB)
        
        # Calculate the elapsed time and sleep for the remainder of the interval

        sleep_time = max(0, interval - elapsed_time)  # Ensure non-negative sleep time
        if (sleep_time == 0):
            print("slow loop cant keep up. please lower frequency or optimize better")
        time.sleep(sleep_time)

if __name__ == "__main__":
    main()
