import numpy as np
import random
import heapq
import os
import time
import matplotlib.pyplot as plt
from helper import *
from visionsystem import VisionSystem


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
        self.rng_ratio = 0.01
        self.nb_max_point = 1000
        self.pixcel_iteration = 5
        self.points_distance = int(80 / self.pixel_size_mm)
        self.pointRadius = int(80 / self.pixel_size_mm)  # minimum distance from other points and obstacles(~80mm)
        self.wall_Radius = int(80 / self.pixel_size_mm)
        self.lineRadius = int(55 / self.pixel_size_mm)  # minimum distance from other points and obstacles(~55mm)
        self.init_enhanced_grid = True
        self.enhanced_grid = []

    def is_valid_point(self, point, placed_points):
        """
        Check if the point is valid (i.e., not on an obstacle, not too close to other points).
        :param point: Tuple (x, y) for the point.
        :param grid: Occupancy grid.
        :param placed_points: List of points already placed.
        :return: Boolean indicating whether the point is valid.
        """
        x, y = point

        # Check that the point is not too close to any other placed point
        for placed_point in placed_points:
            if np.linalg.norm(np.array(placed_point) - np.array(point)) < self.points_distance:
                return False

        return True

    def is_line_colliding(self, start, end):
        """
        Check if the straight line between the two points intersects an obstacle in the occupancy grid
        or is within a specified radius of an obstacle.

        :param start: Tuple (x1, y1) representing the start point.
        :param end: Tuple (x2, y2) representing the end point.
        :param occupancy_grid: 2D numpy array (grid) where True represents occupied pixels.
        :return: Boolean value, True if the line collides with an obstacle or comes too close, False otherwise.
        """
        x1, y1 = start
        x2, y2 = end
        x1 = int(x1)
        x2 = int(x2)
        y1 = int(y1)
        y2 = int(y2)

        # Calculate the differences in x and y
        dx = x2 - x1
        dy = y2 - y1
        slope = dy
        if dx != 0:
            slope = slope / dx  # Calculate the slope of the line

        if (abs(slope) <= 1):  # Iter x if small slope
            if (x1 > x2):  # Ensure that x1, y1 is the left-most point
                x1, x2 = x2, x1
                y1, y2 = y2, y1
            for x in range(int(x1), int(x2) + 1,
                           self.pixcel_iteration):  # Loop over the x range, calculate y for each x
                y = y1 + slope * (x - x1)
                y_int = int(round(y))
                if (self.enhanced_grid[y_int, x]):
                    #print("A y: " + str(y_int)+" x "+str(x))
                    return True

        if (abs(slope) > 1):  # Iter y if slope too stiff
            if (y1 > y2):  # Ensure that x1, y1 is the left-most point
                x1, x2 = x2, x1
                y1, y2 = y2, y1
            for y in range(int(y1), int(y2) + 1,
                           self.pixcel_iteration):  # Loop over the y range, calculate x for each y
                x = ((y - y1) / slope) + x1
                x_int = int(round(x))
                if (self.enhanced_grid[y, x_int]):
                    #print("B y: " + str(y)+" x_int "+str(x_int))
                    return True

        # Handle edge cases for vertical or horizontal lines
        if dx == 0:  # Vertical line
            for y in range(int(min(y1, y2)), int(max(y1, y2)) + 1, self.pixcel_iteration):
                if (self.enhanced_grid[y, x1]):
                    #print("C y: " + str(y)+" x "+str(x1))
                    return True
            return False

        if dy == 0:  # Horizontal line
            for x in range(int(min(x1, x2)), int(max(x1, x2)) + 1, self.pixcel_iteration):
                if (self.enhanced_grid[y1, x]):
                    #print("D y: " + str(y1)+" x "+str(x))
                    return True
            return False

        return False

    def compute_angles_for_waypoints(self, waypoints):
        """
        Computes the angle (theta) between consecutive waypoints and updates the last value of each waypoint with the computed theta.
        The angle is defined as the clockwise rotation from the positive x-axis.

        :param waypoints: List of numpy arrays where each array is of the form [x, y, theta].
        :return: List of updated waypoints with computed theta values.
        """
        if len(waypoints)>1:
            updated_waypoints = []

            for i in range(len(waypoints) - 1):
                # Current and next waypoints
                current = waypoints[i]
                next_wp = waypoints[i + 1]

                # Compute the delta x and delta y
                delta_x = next_wp[0] - current[0]
                delta_y = next_wp[1] - current[1]

                # Compute the angle (theta) using arctan2, adjusting to clockwise from x-axis
                theta = np.arctan2(-delta_y, delta_x)

                # Update the current waypoint with the computed angle
                updated_waypoints.append(np.array([current[0]*self.pixel_size_mm, current[1]*self.pixel_size_mm, theta]))

            # Append the last waypoint with the same theta as the second-to-last one
            last_theta = updated_waypoints[-1][2] if updated_waypoints else 0
            updated_waypoints.append(np.array([waypoints[-1][0]*self.pixel_size_mm, waypoints[-1][1]*self.pixel_size_mm, last_theta]))
        else:
            updated_waypoints = waypoints
        return updated_waypoints

    def compute_enhance_grid(self, occupancy_grid):
        """
        Enhances the occupancy grid by marking a disc-shaped neighborhood of True
        values around each True cell in the original grid.

        Args:
            occupancy_grid (np.ndarray): A 2D binary array representing the occupancy grid.

        Returns:
            np.ndarray: A 2D binary array with the enhanced grid.
        """
        # start_time_grid= time.time()
        # Create the enhanced grid initialized to False
        enhancing_grid = np.zeros_like(occupancy_grid, dtype=bool)

        # Get the shape of the grid
        rows, cols = occupancy_grid.shape

        # Precompute a mask for a disc of radius `lineRadius`
        radius = self.wall_Radius
        diameter = 2 * radius + 1
        y, x = np.ogrid[-radius:radius + 1, -radius:radius + 1]
        disc_mask = (x ** 2 + y ** 2 <= radius ** 2)

        # Loop over the grid and expand the True values using the disc mask
        for r in range(rows):
            for c in range(cols):
                if occupancy_grid[r, c]:
                    # Calculate the bounding box for the disc neighborhood
                    row_min = max(0, r - radius)
                    row_max = min(rows, r + radius + 1)
                    col_min = max(0, c - radius)
                    col_max = min(cols, c + radius + 1)

                    # Get the part of the disc mask that fits within the grid boundaries
                    disc_row_start = max(0, radius - r)
                    disc_row_end = diameter - max(0, r + radius + 1 - rows)
                    disc_col_start = max(0, radius - c)
                    disc_col_end = diameter - max(0, c + radius + 1 - cols)

                    # Apply the mask to the enhancing grid
                    enhancing_grid[row_min:row_max, col_min:col_max] |= \
                        disc_mask[disc_row_start:disc_row_end, disc_col_start:disc_col_end]

        return enhancing_grid

    def get_waypoints(self, occupancy_grid, start, target):
        """
        Get waypoints by randomly placing points on the grid where there is no obstacle in a given radius not an other
        point then apply A* to find the path.
        :param occupancy_grid: Occupancy grid (True for obstacles, False for free space).
        :param start: Start position (x, y, orientation).
        :param target: Target position (x, y, orientation).
        :return: List of waypoints.
        """
        if (self.init_enhanced_grid):
            self.enhanced_grid = self.compute_enhance_grid(occupancy_grid)
            self.init_enhanced_grid = False

        width, height = occupancy_grid.shape
        free_space = np.argwhere(self.enhanced_grid == 0)  # Find all free space pixels
        num_free_pixels = len(free_space)

        # Calculate the target number of points to place (5% of free space but cap to 1000)
        points_to_place = min(int(self.rng_ratio * num_free_pixels), self.nb_max_point)

        placed_points = []
        # Start and target positions (manually added)
        scaled_start = tuple(start[:2]/self.pixel_size_mm)
        scaled_target = tuple(target[:2]/self.pixel_size_mm)
        placed_points.append(scaled_start)
        placed_points.append(scaled_target)

        # Place random points
        timeout = 0
        while len(placed_points) < points_to_place + 2:  # Include start and target
            # Randomly pick a point from free space
            timeout = timeout + 1
            if (timeout > points_to_place * 2):
                break  # in case its not possible to place all the point dont block the system
            y, x = random.choice(free_space)  # value are unpacked in the wrong order for some reason
            if self.is_valid_point((x, y), placed_points):
                placed_points.append((x, y))  # if a point taken at random is in a valid place
                # not too close to an obstacle nor another point.

        # Run A* to find the path from start to target through the placed points
        waypoints = self.a_star(scaled_start, scaled_target, placed_points)
        retour = []  # to see the point placed at random on the map instead of the waypoints :
        #for p in placed_points:
        #   retour.append((p[0]*self.pixel_size_mm, p[1]*self.pixel_size_mm, 0))
        #return retour
        #plot_robot_grid(self.enhanced_grid, 1, robot, robot, robot, goal, waypoints)  # to see the enhanced grid
        return self.compute_angles_for_waypoints(waypoints)  # the angle are a bonus and simplify teh robot control.

    def a_star(self, startpoint, goalpoint, available_nodes):
        """
        Perform A* search to find a path between start and goal points, considering obstacles and the placed points.
        :param grid: Occupancy grid (True for obstacles, False for free space).
        :param start: Start point (x, y, theta).
        :param goal: Goal point (x, y, theta).
        :param available_nodes: List of valid nodes to connect (x, y).
        :return: List of waypoints (sequence of points).
        """
        start = tuple(startpoint[:2])
        goal = tuple(goalpoint[:2])
        # Open list: stores nodes to be evaluated
        open_list = []
        # Closed list: stores nodes that have been fully evaluated
        closed_list = set()

        # Initialize the start node
        g_start = 0  # The cost from the start node to itself is 0
        h_start = np.linalg.norm(np.array(start) - np.array(goal))  # Heuristic from the start to goal
        f_start = g_start + h_start

        # Push start node to open list
        heapq.heappush(open_list, (f_start, start))  # Priority queue based on f = g + h
        came_from = {}  # To reconstruct the path
        g_costs = {start: g_start}  # Store g cost for each node

        while open_list:
            _, current = heapq.heappop(open_list)

            # If the current node is the goal, reconstruct the path
            if current == goal:
                path = []
                while current in came_from:
                    path.append(np.array([current[0], current[1], 0]))
                    current = came_from[current]
                path.append(startpoint)
                return path[::-1]  # Return the reversed path (start to goal)

            closed_list.add(current)  # Add current node to closed list

            # Evaluate each neighboring node (available nodes) sorted by distance
            neighbors = sorted(available_nodes, key=lambda n: np.linalg.norm(np.array(n) - np.array(current)),
                               reverse=False)
            for neighbor in neighbors:
                # If the neighbor is in closed list, skip it
                if neighbor in closed_list:
                    continue

                # Check if the line between current and neighbor collides with an obstacle
                if self.is_line_colliding(current, neighbor):
                    #print("skipped : "+str(current)+" trying to reach"+str(neighbor))
                    continue  # Skip this neighbor if collision detected

                # Calculate the g score for the neighbor
                tentative_g = g_costs[current] + np.linalg.norm(np.array(neighbor) - np.array(current))

                # If the neighbor is not in open_list or we found a better path, update it

                if neighbor not in g_costs or tentative_g < g_costs[neighbor]:
                    g_costs[neighbor] = tentative_g
                    h = np.linalg.norm(np.array(neighbor) - np.array(goal))
                    f = tentative_g + h

                    heapq.heappush(open_list, (f, neighbor))  # Add neighbor to open list

                    came_from[neighbor] = current  # Record the parent node to reconstruct the path
        print("No Path Found")
        return []  # Return an empty list if no path found


if __name__ == "__main__":
    # Load and process the occupancy grid
    path = os.path.join("testData", "mapWithBlackObstacle1.txt") #mapOneIsland2
    matrix = np.loadtxt(path, dtype=int)
    occupancyGrid = matrix.astype(bool)
    vision = VisionSystem(use_camera=True)
    occupancyGrid = vision.generate_occupancy_grid()
    # Define the goal and robot states
    goal = np.array([36, 294, np.pi / 6])  # slightly different than the default camera one
    robot = np.array([463,382, np.pi / 6])  # slightly different than the default camera one
    goal = vision.get_goal_position()
    robot = vision.get_robot_position()
    print("Robot:", robot)
    print("Goal:", goal)

    # Initialize robot speed (assumed to be stationary)
    robotSpeedFromEncoder = np.array([0, 0, 0])  # no speed

    # Initialize the planner
    planner = PathPlanner()

    # Time the waypoint calculation
    start_time = time.time()
    waypoints = planner.get_waypoints(occupancyGrid, robot, goal)
    print("number of waypoints "+str(len(waypoints)))
    print("waypoints "+str(waypoints))
    end_time = time.time()
    occupancyGrid = planner.enhanced_grid
    # Plot and finalize
    plot_robot_grid(occupancyGrid, vision.get_pixel_side_mm(), robot, robot, robot, goal, waypoints)
    print("Waypoint calculation complete.")
    print(f"Elapsed time: {end_time - start_time:.4f} seconds")
