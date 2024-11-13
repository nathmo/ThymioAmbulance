import threading
import time
from robotmovement import RobotMovement


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
    # Create an instance of RobotMovement
    robot = RobotMovement()
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

        print("Main loop running...")
        time.sleep(1)  # Adjust the main loop's frequency as needed

if __name__ == "__main__":
    main()
