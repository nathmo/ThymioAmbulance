class RobotMovement:
    def __init__(self):
        # Initialize motors, sensors, and encoders
        self.waypoints = [np.array([0, 0, 0])] # list of np.array
        self.position = np.array([0, 0, 0])
        self.speed = np.array([0, 0, 0])

    def set_waypoints(self, waypoints): # Beware, this class is inside a thread you need to use a mutex !!!
        # same format as get_waypoints
        # update the global list of waypoint to follow

    def get_waypoints(self, waypoints): # Beware, this class is inside a thread you need to use a mutex !!!
        # return the global list of waypoint to follow minus the one that where reached aldready
        waypoints = [
            np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians) clockwise rotation from x axis
            np.array([200, 250, np.pi / 3]),  # x=200mm, y=250mm, direction=60° (π/3 radians)
            np.array([300, 350, np.pi / 2])   # x=300mm, y=350mm, direction=90° (π/2 radians)
        ]
        return waypoints

    def get_position(self):# Beware, this class is inside a thread you need to use a mutex !!!
        # Return estimated position from encoders
        # same format as for set_position

    def set_position(self, kalman_position):# Beware, this class is inside a thread you need to use a mutex !!!
        # Update the robot’s position based on Kalman filter results
        return np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians)

    def get_speed(self): # there is no need to set the speed from kalman, the motor know better
        # Return the speed+angular speed from encoders
        # Beware, this class is inside a thread you need to use a mutex !!!
        return np.array([10, 15, np.pi / 60]),  # x=10mm/s, y=15mm/s, direction=3°/s (π/60 radians/second)

    def update(self):# this function is called every 10 ms for you to do what you have to
                     # (avoid obstacle, go to the nearest waypoint)
        return None