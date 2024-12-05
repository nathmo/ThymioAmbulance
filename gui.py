import tkinter as tk
from tkinter import filedialog, scrolledtext
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
import threading
import cv2
import numpy as np
from time import sleep
from robotmovement import RobotMovement
from visionsystem import VisionSystem
from sensorfusion import SensorFusion
from pathplanner import PathPlanner


class RobotSimulationApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Simulation")

        # Variables for simulation state
        self.simulate_robot = tk.BooleanVar(value=True)
        self.simulate_camera = tk.BooleanVar(value=True)  # Start with camera simulation enabled
        self.simulate_movement = tk.BooleanVar()
        self.simulate_pathplanning = tk.BooleanVar()
        self.selected_file = tk.StringVar()
        self.camera_id = tk.IntVar(value=0)  # Default cameraID is 0
        self.simulation_running = False

        # Frame for buttons
        button_frame = tk.Frame(root)
        button_frame.pack(side=tk.LEFT, fill=tk.Y)

        # Robot simulation toggle
        self.robot_button = tk.Checkbutton(button_frame, text="Simulate Robot", variable=self.simulate_robot,
                                           onvalue=True, offvalue=False)
        self.robot_button.pack(pady=5)

        # Camera simulation toggle
        self.camera_button = tk.Checkbutton(button_frame, text="Simulate Camera", variable=self.simulate_camera,
                                            onvalue=True, offvalue=False, command=self.check_start_button)
        self.camera_button.pack(pady=5)

        # Movement simulation toggle
        self.movement_button = tk.Checkbutton(button_frame, text="Simulate Movement", variable=self.simulate_movement,
                                              onvalue=True, offvalue=False)
        self.movement_button.pack(pady=5)

        # Path Planning toggle
        self.pathplanning_button = tk.Checkbutton(button_frame, text="Path Planning", variable=self.simulate_pathplanning,
                                                  onvalue=True, offvalue=False)
        self.pathplanning_button.pack(pady=5)

        # File selection button
        tk.Button(button_frame, text="Select Image", command=self.select_file).pack(pady=5)

        # Input field for camera ID
        tk.Label(button_frame, text="Enter Camera ID (0-6):").pack(pady=5)
        self.camera_id_entry = tk.Entry(button_frame, textvariable=self.camera_id)
        self.camera_id_entry.pack(pady=5)
        self.camera_id_entry.bind("<FocusOut>", self.validate_camera_id)

        # Start Simulation button
        self.start_button = tk.Button(button_frame, text="Start Simulation", command=self.start_simulation, state="disabled")
        self.start_button.pack(pady=5)

        # Label for file selection
        self.file_label = tk.Label(button_frame, text="No file selected", wraplength=200)
        self.file_label.pack(pady=5)

        # Console output
        self.console_output = scrolledtext.ScrolledText(button_frame, wrap=tk.WORD, height=20, width=50,
                                                        state="disabled")
        self.console_output.pack(pady=5)

        # Figure and Matplotlib
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim(0, 100)
        self.ax.set_ylim(0, 100)
        self.ax.set_title("Robot Simulation")

        canvas = FigureCanvasTkAgg(self.fig, root)
        canvas_widget = canvas.get_tk_widget()
        canvas_widget.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        self.canvas = canvas

        # State variables
        self.running = True
        self.waypoints = []  # Initialize waypoints

    def select_file(self):
        file_path = filedialog.askopenfilename()
        if file_path:
            self.selected_file.set(file_path)
            self.file_label.config(text=f"Selected: {file_path}")
            self.check_start_button()

    def validate_camera_id(self, event=None):
        """Validate the camera ID to ensure it's between 0 and 6."""
        try:
            value = self.camera_id.get()
            if value < 0 or value > 6:
                raise ValueError
        except ValueError:
            self.update_console("Camera ID must be an integer between 0 and 6.")
            self.camera_id.set(0)  # Reset to default value

    def check_start_button(self):
        """Enable Start Simulation button if the conditions are met."""
        if (not self.simulate_camera.get()) or self.selected_file.get() :
            self.start_button.config(state="normal")
        else:
            self.start_button.config(state="disabled")

    def start_simulation(self):
        if not self.simulation_running:
            self.simulation_running = True
            self.update_console("Starting Simulation...")
            threading.Thread(target=simulation_thread, args=(self,), daemon=True).start()

    def update_console(self, message):
        self.console_output.configure(state="normal")
        self.console_output.insert(tk.END, message + "\n")
        self.console_output.configure(state="disabled")
        self.console_output.yview(tk.END)

    def update_plot(self, robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, waypoints,
                    frame, scale):
        self.ax.clear()
        height, width = frame.shape[:2]
        self.ax.imshow(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB), extent=[0, width * scale, 0, height * scale], origin='lower')

        # Plot positions
        self._plot_position(robotPosFromEncoder, 'blue', 'Robot Encoder Position')
        self._plot_position(robotPosFromCamera, 'green', 'Robot Camera Position')
        self._plot_position(robotPosFromFusion, 'purple', 'Robot Fusion Position')
        self._plot_position(goalPosFromCamera, 'red', 'Goal Position')

        # Plot waypoints
        for idx, waypoint in enumerate(waypoints):
            x, y, theta = waypoint
            self.ax.plot(x, y, 'x', color='cyan')
            self.ax.annotate(f'WP{idx}', (x, y), textcoords="offset points",
                             xytext=(5, 5), ha='center', color='cyan')

        self.canvas.draw()

    def _plot_position(self, pos, color, label):
        if pos is not None:
            x, y, theta = pos
            self.ax.plot(x, y, 'o', color=color, label=label)
            self.ax.arrow(x, y, np.cos(theta) * 50, np.sin(theta) * 50, head_width=5, head_length=5, fc=color, ec=color)
            self.ax.legend()

    def close(self):
        self.running = False
        self.root.destroy()


# Thread-safe function to update the GUI
def simulation_thread(app):
    robot = RobotMovement(debug=app.simulate_robot.get())
    vision = VisionSystem(use_camera=(not app.simulate_camera.get()), cameraID=app.camera_id.get(), image_path=app.selected_file.get())
    sensorfusion = SensorFusion()
    pathplanning = PathPlanner(pixel_size_mm=vision.get_pixel_side_mm())

    robot.set_position(vision.get_robot_position())
    while app.running:
        robotPosFromCamera = vision.get_robot_position()
        goalPosFromCamera = vision.get_goal_position()
        occupancyGrid = vision.generate_occupancy_grid()

        robotPosFromEncoder = robot.get_position()
        robotSpeedFromEncoder = robot.get_speed()
        if not(np.array_equal(robotPosFromCamera, np.array([0.0, 0.0, 0.0])) or np.array_equal(goalPosFromCamera, np.array([0.0, 0.0, 0.0]))):
            robotPosFromFusion = sensorfusion.get_estimated_position(robotPosFromEncoder, robotSpeedFromEncoder, robotPosFromCamera)
        else:
            robotPosFromFusion = np.array([0.0, 0.0, 0.0])
        #robot.set_position(robotPosFromFusion)
        # Update waypoints only if Path Planning is enabled or if waypoints are empty
        if app.simulate_pathplanning.get() or not app.waypoints:
            app.waypoints = pathplanning.get_waypoints(occupancyGrid, robotPosFromFusion, goalPosFromCamera)
            robot.set_waypoints(app.waypoints)
            previousNumberOfWaypoints = len(robot.get_waypoints())
            app.update_console("Updated robot waypoints.")

        frame = vision.get_frame()
        app.update_plot(robotPosFromEncoder, robotPosFromCamera, robotPosFromFusion, goalPosFromCamera, app.waypoints,
                        frame, vision.get_pixel_side_mm())
        robot.update()

        if len(robot.get_waypoints()) < previousNumberOfWaypoints:
            previousNumberOfWaypoints = len(robot.get_waypoints())
            app.update_console("waypoint Reached")
        sleep(1)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulationApp(root)

    root.protocol("WM_DELETE_WINDOW", app.close)
    root.mainloop()
