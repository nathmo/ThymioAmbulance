
import numpy as np
import matplotlib.pyplot as plt
import time
import cv2
import os

class Visualisation:
    def __init__(self, gridSquareSizeMM, occupancyGrid=None):
        """
        Initialize the dynamic visualizer with scaling for the occupancy grid.
        """
        self.gridSquareSizeMM = gridSquareSizeMM
        self.occupancyGrid = occupancyGrid
        self.cvImage = None
        if self.occupancyGrid is not None:
            self.height, self.width = self.occupancyGrid.shape
        else:
            self.height = self.width = 0
        self.occupancy_height_mm = self.height * gridSquareSizeMM

        # Setup the figure and axis
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self._setup_plot()

    def _setup_plot(self):
        """
        Initial setup for plot limits and labels.
        """
        if self.occupancyGrid is not None:
            self.ax.set_xlim([0, self.width * self.gridSquareSizeMM])
            self.ax.set_ylim([0, self.height * self.gridSquareSizeMM])
        else:
            self.ax.set_xlim([0, 1])  # Placeholder values if no occupancy grid is provided
            self.ax.set_ylim([0, 1])

        self.ax.set_xlabel("X position (mm)")
        self.ax.set_ylabel("Y position (mm)")
        self.ax.set_title("Occupancy Grid and Robot Positions")

    def update_background(self, occupancyGrid, cvImage):
        """
        Update the background with an occupancy grid and an OpenCV image.
        The image is displayed with transparency over the grid.
        """
        self.occupancyGrid = occupancyGrid
        self.cvImage = cvImage

        # Get dimensions from occupancy grid
        self.height, self.width = occupancyGrid.shape

        # Convert OpenCV image to RGB for matplotlib
        cvImageRGB = cv2.cvtColor(cvImage, cv2.COLOR_BGR2RGB)

        # Clear current background
        self.ax.clear()

        # Re-setup plot with new limits
        self._setup_plot()

        # Display the occupancy grid
        self.ax.imshow(occupancyGrid, cmap='gray_r', origin='upper',
                       extent=[0, self.width * self.gridSquareSizeMM,
                               0, self.height * self.gridSquareSizeMM], alpha=0.5)

        # Overlay the OpenCV image with transparency
        self.ax.imshow(cvImageRGB, origin='upper',
                       extent=[0, self.width * self.gridSquareSizeMM,
                               0, self.height * self.gridSquareSizeMM], alpha=0.5)

        # Redraw canvas to force rendering
        self.fig.canvas.draw_idle()


    def _plot_position(self, position, color, label):
        """
        Plot a position with orientation on the grid.
        """
        x, y, theta = position
        y = self.height * self.gridSquareSizeMM - y  # Adjust for top-left origin
        self.ax.plot(x, y, 'o', color=color, label=label)
        self.ax.arrow(x, y, np.cos(theta) * 10, np.sin(theta) * 10,
                      head_width=5, head_length=5, fc=color, ec=color)

    def update_plot(self, robotPosFromEncoder, robotPosFromCamera,
                    robotPosFromFusion, goalPosFromCamera, waypoints):
        """
        Update the plot dynamically with the latest data and save it as a PNG.
        """
        # Clear the axis
        self.ax.cla()  # Clears the current axis

        # Re-setup the plot (otherwise, it will disappear after clearing)
        self._setup_plot()

        # Plot robot positions and goal
        self._plot_position(robotPosFromEncoder, 'blue', 'Robot Encoder Position')
        self._plot_position(robotPosFromCamera, 'green', 'Robot Camera Position')
        self._plot_position(robotPosFromFusion, 'purple', 'Robot Fusion Position')
        self._plot_position(goalPosFromCamera, 'red', 'Goal Position')

        # Plot waypoints
        for idx, waypoint in enumerate(waypoints):
            x, y, theta = waypoint
            y = self.height * self.gridSquareSizeMM - y  # Adjust for top-left origin
            self.ax.plot(x, y, 'x', color='orange')
            self.ax.annotate(f'WP{idx}', (x, y), textcoords="offset points",
                             xytext=(5, 5), ha='center', color='orange')

        # Refresh legend
        self.ax.legend(loc='upper right')
        plt.pause(0.01)  # Allow time for the plot to refresh

        # Save the plot as PNG with a Unix timestamp-based filename
        timestamp = int(time.time())
        filename = f"liveVisu_{timestamp}.png"
        plt.savefig(os.path.join("result", filename))

        # Optionally display the plot
        # plt.show()  # Uncomment this if you want to display the plot interactively

    def import_matrix_from_txt(filename):
        """
        Imports a NumPy boolean matrix from a text file.

        Args:
            filename (str): The name of the input file.

        Returns:
            np.ndarray: The imported boolean matrix.
        """
        # Load the matrix and convert integers back to boolean
        matrix = np.loadtxt(filename, dtype=int)
        return matrix.astype(bool)

    def plot_robot_grid(occupancyGrid, gridSquareSizeMM, robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, waypoints):
        fig, ax = plt.subplots(figsize=(8, 8))

        # Display the occupancy grid as a black and white grid
        grid_size = gridSquareSizeMM  # each cell is n mm with and tall
        height, width = occupancyGrid.shape
        ax.imshow(occupancyGrid, cmap='gray_r', origin='upper', extent=[0, width * grid_size, 0, height * grid_size])
        # Obstacle (value set to true) are in white

        # Plot robot and goal positions with their orientations
        def plot_position_with_orientation(position, color, label, plot_height):
            x, y, theta = position
            # Adjust y to assume origin is at the upper-left corner
            y = plot_height - y
            ax.plot(x, y, 'o', color=color, label=label)
            ax.arrow(x, y, np.cos(theta) * 10, np.sin(theta) * 10, head_width=5, head_length=5, fc=color, ec=color)

        # Plot positions with the adjusted y-coordinate
        plot_position_with_orientation(robotPosFromEncoder, 'blue', 'Robot Encoder Position', height*gridSquareSizeMM)
        plot_position_with_orientation(robotPosFromCamera, 'green', 'Robot Camera Position', height*gridSquareSizeMM)
        plot_position_with_orientation(robotPosFromFusion, 'purple', 'Robot Fusion Position', height*gridSquareSizeMM)
        plot_position_with_orientation(goalPosFromCamera, 'red', 'Goal Position', height*gridSquareSizeMM)

        # Plot waypoints
        for idx, waypoint in enumerate(waypoints):
            x, y, theta = waypoint
            ax.plot(x, y, 'x', color='orange')
            ax.annotate(f'WP{idx}', (x, y), textcoords="offset points", xytext=(5, 5), ha='center', color='orange')

        # Set plot limits and labels
        ax.set_xlim([0, width * grid_size])
        ax.set_ylim([0, height * grid_size])
        ax.set_xlabel("X position (mm)")
        ax.set_ylabel("Y position (mm)")
        ax.set_title("Occupancy Grid and Robot Positions")
        ax.legend(loc='upper right')

        plt.savefig(os.path.join("result", str(int(time.time()))+".png"))
        plt.show()