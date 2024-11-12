class PathPlanner:
    #
    def get_waypoints(occupancy_grid, start, target):
        # A* algorithm to calculate path
        # Return a list of waypoints to follow, read the class of work package 1 + 2 to have the specific of the format. (robot + VisionSystem)
        start = np.array([100, 150, np.pi / 6]) # both start and target are defined as numpy array with the first component
        target = np.array([100, 150, np.pi / 6]) # being the x axis, second is y, and third is the angle relative to x axis in radian
                                                 # clockwise rotation
        waypoints = [
            np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians)
            np.array([200, 250, np.pi / 3]),  # x=200mm, y=250mm, direction=60° (π/3 radians)
            np.array([300, 350, np.pi / 2])   # x=300mm, y=350mm, direction=90° (π/2 radians)
        ]
        return waypoints