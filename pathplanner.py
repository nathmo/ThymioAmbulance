import numpy as np
import random
import heapq
import os
import time
import matplotlib.pyplot as plt
from helper import *

class PathPlanner:
    def __init__(self, pixel_size_mm=4):
        """
        Initialize PathPlanner with the size of each pixel in millimeters and the minimum radius for random point placement.
        :param pixel_size_mm: Size of each pixel in millimeters.
        :param n: Minimum distance (in pixels) between points and obstacles.
        """
        self.pixel_size_mm = pixel_size_mm
        self.pointRadius = int(80/self.pixel_size_mm)  # minimum distance from other points and obstacles(~80mm)
        self.lineRadius = int(55/self.pixel_size_mm)  # minimum distance from other points and obstacles(~55mm)

    def is_valid_point(self, point, grid, placed_points):
        """
        Check if the point is valid (i.e., not on an obstacle, not too close to other points).
        :param point: Tuple (x, y) for the point.
        :param grid: Occupancy grid.
        :param placed_points: List of points already placed.
        :return: Boolean indicating whether the point is valid.
        """
        x, y = point

        # Check if the point itself is on an obstacle or too close to an obstacle
        if self.is_within_radius(point, grid, self.pointRadius):
            return False

        # Check that the point is not too close to any other placed point
        for placed_point in placed_points:
            if np.linalg.norm(np.array(placed_point) - np.array(point)) < self.pointRadius:
                return False

        return True

    def is_line_colliding(self, start, end, occupancy_grid):
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

        # Ensure that x1, y1 is the left-most point
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1

        # Calculate the differences in x and y
        dx = x2 - x1
        dy = y2 - y1

        # Handle edge cases for vertical or horizontal lines
        if dx == 0:  # Vertical line
            for y in range(int(min(y1, y2)), int(max(y1, y2)) + 1):
                if self.is_within_radius((x1, y), occupancy_grid, self.lineRadius):
                    return True
            return False

        if dy == 0:  # Horizontal line
            for x in range(int(min(x1, x2)), int(max(x1, x2)) + 1):
                if self.is_within_radius((x, y1), occupancy_grid, self.lineRadius):
                    return True
            return False

        # Calculate the slope of the line
        slope = dy / dx

        # Loop over the x range, calculate y for each x, and check for obstacles within radius
        for x in range(int(x1), int(x2) + 1):
            y = y1 + slope * (x - x1)
            y_int = int(round(y))

            if self.is_within_radius((x, y_int), occupancy_grid, self.lineRadius):
                return True

        return False

    def is_within_radius(self, point, grid, radius):
        """
        Check if any obstacle is present within the square region around a point.

        :param point: Tuple (x, y) representing the point.
        :param grid: 2D numpy array representing the occupancy grid.
        :param radius: Radius defining the size of the square region.
        :return: Boolean value, True if an obstacle is within the square region, False otherwise.
        """
        x, y = map(int, point)  # Convert point to integer indices

        # Define the bounds of the square region
        x_min = max(0, x - radius)
        x_max = min(grid.shape[1], x + radius + 1)
        y_min = max(0, y - radius)
        y_max = min(grid.shape[0], y + radius + 1)

        # Extract the square region of interest
        region = grid[y_min:y_max, x_min:x_max]

        # Check if any value in the region is True
        return np.any(region)

    def compute_angles_for_waypoints(self, waypoints):
        """
        Computes the angle (theta) between consecutive waypoints and updates the last value of each waypoint with the computed theta.
        The angle is defined as the clockwise rotation from the positive x-axis.

        :param waypoints: List of numpy arrays where each array is of the form [x, y, theta].
        :return: List of updated waypoints with computed theta values.
        """
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
            updated_waypoints.append(np.array([current[0], current[1], theta]))

        # Append the last waypoint with the same theta as the second-to-last one
        last_theta = updated_waypoints[-1][2] if updated_waypoints else 0
        updated_waypoints.append(np.array([waypoints[-1][0], waypoints[-1][1], last_theta]))

        return updated_waypoints

    def get_waypoints(self, occupancy_grid, start, target):
        """
        Get waypoints by randomly placing points on the grid and applying A* to find the path.
        :param occupancy_grid: Occupancy grid (True for obstacles, False for free space).
        :param start: Start position (x, y, orientation).
        :param target: Target position (x, y, orientation).
        :return: List of waypoints.
        """
        # Define some variables
        width, height = occupancy_grid.shape
        free_space = np.argwhere(occupancy_grid == 0)  # Find all free space pixels
        num_free_pixels = len(free_space)
        # Calculate the target number of points to place (5% of free space but cap to 1000)
        target_num_points = min(int(0.05 * num_free_pixels),1000)

        placed_points = []
        points_to_place = target_num_points

        # Start and target positions (manually added)
        placed_points.append(tuple(start[:2]))
        placed_points.append(tuple(target[:2]))

        # Place random points
        timeout = 0
        while len(placed_points) < points_to_place + 2:  # Include start and target
            # Randomly pick a point from free space
            timeout = timeout + 1
            if(timeout>points_to_place*2):
                break
            y, x = random.choice(free_space) # value are unpacked in the wrong order for some reason
            if self.is_valid_point((x, y), occupancy_grid, placed_points):
                placed_points.append((x, y))
        # Run A* to find the path from start to target through the placed points
        waypoints = self.a_star(occupancy_grid, start, target, placed_points)
        retour = []
        for p in placed_points:
            retour.append((p[0],p[1], 0))
        return self.compute_angles_for_waypoints(waypoints)

    def a_star(self, grid, startpoint, goalpoint, available_nodes):
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
        h_start = np.linalg.norm(np.array(start) - np.array(goal)) # Heuristic from the start to goal
        f_start = g_start + h_start

        # Push start node to open list
        heapq.heappush(open_list, (f_start, start))  # Priority queue based on f = g + h
        came_from = {}  # To reconstruct the path
        g_costs = {start: g_start}  # Store g cost for each node

        while open_list:
            # Get the node with the lowest f value
            _, current = heapq.heappop(open_list)

            # If the current node is the goal, reconstruct the path
            if current == goal:
                path = []
                while current in came_from:
                    path.append(np.array([current[0],current[1], 0]))
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
                if self.is_line_colliding(current, neighbor, grid):
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

        return []  # Return an empty list if no path found



if __name__ == "__main__":
    # Load and process the occupancy grid
    path = os.path.join("testData", "mapWithBlackObstacle1.txt") #mapOneIsland2
    matrix = np.loadtxt(path, dtype=int)
    occupancyGrid = matrix.astype(bool)

    # Define the goal and robot states
    goal = np.array([30, 30, np.pi / 6])  # slightly different than the default camera one
    robot = np.array([460,400, np.pi / 6])  # slightly different than the default camera one
    print("Robot:", robot)
    print("Goal:", goal)

    # Initialize robot speed (assumed to be stationary)
    robotSpeedFromEncoder = np.array([0, 0, 0])  # no speed

    # Initialize the planner
    planner = PathPlanner()

    # Time the waypoint calculation
    start_time = time.time()
    waypoints = planner.get_waypoints(occupancyGrid, robot, goal)
    print("waypoints "+str(waypoints))
    end_time = time.time()

    # Plot and finalize
    plot_robot_grid(occupancyGrid, 1, robot, robot, robot, goal, waypoints)
    print("Waypoint calculation complete.")
    print(f"Elapsed time: {end_time - start_time:.4f} seconds")
