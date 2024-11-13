import numpy as np
from threading import Lock

class RobotMovement:
    def __init__(self):
        # Initialize motors, sensors, and encoders
        self.waypoints = [np.array([0, 0, 0])]  # list of np.array for waypoints
        self.position = np.array([0, 0, 0])     # Current position of the robot
        self.speed = np.array([0, 0, 0])        # Current speed of the robot
        self._lock = Lock()  # Mutex lock for thread safety

    def set_waypoints(self, waypoints):
        # Update the global list of waypoints to follow (requires mutex for thread safety)
        with self._lock:
            self.waypoints = waypoints

    def get_waypoints(self):
        # Return the global list of waypoints to follow minus those already reached
        # Requires mutex to ensure thread-safe access
        with self._lock:
            # Example waypoints, replace with the actual dynamic waypoint list as needed
            waypoints = [
                np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians)
                np.array([200, 250, np.pi / 3]),  # x=200mm, y=250mm, direction=60° (π/3 radians)
                np.array([300, 350, np.pi / 2])   # x=300mm, y=350mm, direction=90° (π/2 radians)
            ]
            return waypoints

    def get_position(self):
        # Return the robot's estimated position from encoders
        # Requires mutex for thread-safe access
        with self._lock:
            return self.position

    def set_position(self, kalman_position):
        # Update the robot’s position based on Kalman filter results
        # Requires mutex for thread-safe update
        with self._lock:
            self.position = kalman_position

    def get_speed(self):
        # Return the robot's speed and angular speed from encoders
        # Requires mutex for thread-safe access
        with self._lock:
            return np.array([10, 15, np.pi / 60])  # Example: x=10mm/s, y=15mm/s, rotation=π/60 radians/s

    def update(self):
        # Called every 10 ms to perform necessary tasks
        # (e.g., avoid obstacles, move to the nearest waypoint)
        # Requires mutex for any shared resource modification
        with self._lock:
            # Implement movement, obstacle avoidance, etc.
            pass
