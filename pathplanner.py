import numpy as np
class PathPlanner:
    def __init__(self, pixel_size_mm=4):
        """
        Initialize PathPlanner with the size of each pixel in millimeters and the minimum radius for random point placement.
        :param pixel_size_mm: Size of each pixel in millimeters.
        :param n: Minimum distance (in pixels) between points and obstacles.
        """
        self.pixel_size_mm = pixel_size_mm
        # This approach ensure that the thymio will not hit any obstacle while not having to evaluate a complex
        # bounding box. we take the small radius (with of the robot) along the line because the robot go straight
        # we take the larger radius at each node as the robot will have to turn on itself at theses point and must
        # avoid colliding into obstacle
        self.pointRadius = int(80/self.pixel_size_mm)  # minimum distance from other points and obstacles(~80mm)
        self.lineRadius = int(55/self.pixel_size_mm)  # minimum distance from other points and obstacles(~55mm)

    def get_waypoints(self,occupancy_grid, start, target):
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