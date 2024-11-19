import numpy as np
from threading import Lock
import math
from tdmclient import ClientAsync, aw, thymio
import time

# Constants
BASICSPEED = 100
GAIN = 10
ANGULAR_GAIN = 15

class RobotMovement:
    def __init__(self):
        # Initialize motors, sensors, and encoders
        self.client = ClientAsync()
        self.node = None
        self.waypoints = [np.array([0, 0, 0])]  # list of np.array for waypoints
        self.position = np.array([0, 0, 0])     # Current position of the robot
        self.speed = np.array([0, 0, 0])        # Current speed of the robot
        self._lock = Lock()  # Mutex lock for thread safety

    def set_waypoints(self, waypoints):
        # Update the global list of waypoints to follow (requires mutex for thread safety)
        with self._lock:
            self.waypoints = waypoints

    def connect(self):
        self.node = aw(self.client.wait_for_node())            

    def get_waypoints(self):
        # Return the global list of waypoints to follow minus those already reached
        # Requires mutex to ensure thread-safe access
        with self._lock:
            # Example waypoints, replace with the actual dynamic waypoint list as needed
            # waypoints = [
            #     np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians)
            #     np.array([200, 250, np.pi / 3]),  # x=200mm, y=250mm, direction=60° (π/3 radians)
            #     np.array([300, 350, np.pi / 2])   # x=300mm, y=350mm, direction=90° (π/2 radians)
            # ]
            if ((abs(self.waypoints[0] - self.position)) < 50):
                self.waypoints.pop(0)
            return self.waypoints

    def get_position(self):
        # Return the robot's estimated position from encoders
        # Requires mutex for thread-safe access
        with self._lock:
            return self.position

    def set_position(self, kalman_position):
        # Update the robot’s position based on Kalman filter results
        # Requires mutex for thread-safe update
        with self._lock:
            self.position = (kalman_position)

    def get_speed(self):
        # Return the robot's speed and angular speed from encoders
        # Requires mutex for thread-safe access
        with self._lock:
            return self.speed #np.array([10, 15, np.pi / 60])  # Example: x=10mm/s, y=15mm/s, rotation=π/60 radians/s

    def update(self):
        # Called every 10 ms to perform necessary tasks
        # (e.g., avoid obstacles, move to the nearest waypoint)
        # Requires mutex for any shared resource modification
        with self._lock:
            # Implement movement, obstacle avoidance, etc.
            goal_x, goal_y, goal_theta = self.waypoints[0]
            x, y, theta = self.position

            distance = math.sqrt((goal_x - x) ** 2 + (goal_y - y) ** 2)
            desired_angle = math.atan2(goal_y - y, goal_x - x)
            heading_error = desired_angle - theta
            heading_error = (heading_error + math.pi) % (2 * math.pi) - math.pi


            diff = heading_error * ANGULAR_GAIN
            motor_left_target = BASICSPEED - diff*GAIN
            motor_right_target = BASICSPEED + diff*GAIN   

            self.speed = np.array([motor_left_target, motor_right_target, heading_error])

            aw(self.node.lock())  # Acquire lock explicitly
            aw(self.client.wait_for_status(self.client.NODE_STATUS_READY))
            v = {
            "motor.left.target": [motor_left_target],
            "motor.right.target": [motor_right_target],
            }
            aw(self.node.set_variables(v))
            aw(self.node.unlock())  # Release lock

            if ((abs(self.waypoints[0] - self.position)) < 50):
                self.waypoints.pop(0)

            # Local AVOIDANCE   
