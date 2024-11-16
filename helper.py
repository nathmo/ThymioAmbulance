
import numpy as np
import matplotlib.pyplot as plt
import time
import os

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
    grid_size = gridSquareSizeMM  # each cell is 10mm
    height, width = occupancyGrid.shape
    ax.imshow(occupancyGrid, cmap='gray', origin='lower', extent=[0, width * grid_size, 0, height * grid_size])
    # Obstacle (value set to true) are in white

    # Plot robot and goal positions with their orientations
    def plot_position_with_orientation(position, color, label):
        x, y, theta = position
        ax.plot(x, y, 'o', color=color, label=label)
        ax.arrow(x, y, np.cos(theta) * 10, np.sin(theta) * 10, head_width=5, head_length=5, fc=color, ec=color)

    plot_position_with_orientation(robotPosFromEncoder, 'blue', 'Robot Encoder Position')
    plot_position_with_orientation(robotPosFromCamera, 'green', 'Robot Camera Position')
    plot_position_with_orientation(robotPosFromFusion, 'purple', 'Robot Fusion Position')
    plot_position_with_orientation(goalPosFromCamera, 'red', 'Goal Position')

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