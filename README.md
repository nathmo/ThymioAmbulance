# ThymioAmbulance
This repo contains code and documentation to control the thymio from a laptop using
a ceiling mounted camera to navigate a labyrinthe

# Lore
Pompier/ambulance -> thymio 
- évite voiture sur la route
- GPS (camera)
- Connaitre les routes (obsctacle vu par la camera unlike voiture)
va chercher le patient ASAP 

Position de marker avec une camera
Path finding(astar)
Localisation bruité avec kalman

# Architecture
Main loop :
- get the Camera position of the map and robot and the target (car crash, ciff)
- get the map layout (road line) (Camera)
- get the position from the robot encoder
- use kalmann to improve robot position
- update robot position
- use path planning to find best way (A*)
- update robot movement instruction

Fast loop :
- react to obstacle and avoid,
- follow track given by path planning

Pathplanning :
- use the occupancy matrix to compute the fastest route 

ComputerVision :
- get the image dimmension + calibrate the camera.
- get the robot + goal position
- get the occupancy map

Robot :
deal with the hardware + realtime
- give estimated position
- allow the update of the estimated position
- get a list of waypoint and follow them.

sensorfusion :
- merge the data from the camera + encoder to improve the accuracy.

# Work package 1) -> Seif
python class for robot movement.
this is the fast loop. 
It run in its own thread and handle to robot movement  + communication and local obstacle avoidance. 
It will be fed a list of waypoint from the slower path planning and will follow them unless it's path is blocked and will then go around the obstacle to meet back the waypoint line. This class return the estimated position+speed from the wheel encoder.(when the position is updated from the kalman filter, keep updating it over time from the position delta from the wheel encoder so that when we read back that value later one we know where you are now estimated by the wheel encoder.)

also make the thymio blink in blue like an ambulance.

```
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
```

to ensure this work, try to manually generate a list of waypoint, see how it behave
add a few obstacle along the path, does it work ?
what about the quality of the position estimation ? does it match with the measured position ?
make your code robust and failproof to connection problem to the robot. (expect it and tell the user what to do to solve it) 

(ask chatGPT to generate a main and a thread and call the update function from the thread at 100 Hz/ every 10ms), make sure your mutex work when you update the waypoints/position

# Work package 2) -> Felipe
the goal if to use openCV to find the position and orientation of the objects on the map.
The camera will be look at the floor and can see the whole map.
This module will return the position and orientation of the robot + a NxM matrix with the space occupancy of the map (used for path planning) + the position and orientation of the crash.

```
class VisionSystem:
    def __init__(self):
        # Initialize camera and other parameters

    def is_camera_ready(self):
      return True # or False if the camera is not ready

    def get_robot_position(self):
        # Use OpenCV to detect robot’s position and orientation
        return np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians)

    def get_goal_position(self):
        # Detect and return the crash site position ->
        # the positionning is absolute and explained in generate_occupancy_grid
        return np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians) clockwise rotation from x axis

    def generate_occupancy_grid(self):
        # Create and return NxM occupancy grid based on map layout
        # each square is 0.5-5 cm width (check that its still fast enough while being precise enough
        # 
        # this is an example for the output format, you define the dimension required 
        # so that each square 1-2 cm in width (its easy to read it back)
        # True mean the space is occupied, false mean it's free
        # only set to true the black area of the map (outside the road) but not the thymio or the goal or the
        # small obstacle on the road.
        # The 0,0 coordinate is on the lower left corner
        # the row are along the x coordinate
        # the column are along the y coordinate
        matrix = []

        # Loop over the rows
        for i in range(120):
            # Initialize an empty list to hold the current row
            row = []
    
            # Loop over the columns
            for j in range(150):
                # Randomly choose True or False
                row.append(True if np.random.rand() > 0.5 else False)
    
            # Append the row to the matrix
            matrix.append(row)

        # Convert the list of lists into a NumPy array
        return np.array(matrix)

```
To demonstrate how your system work, use the visualisation helper tool (made by the person in charge of work package 4) that render both the robot/goal and the numpy array for the grid. try to play with the lightning, add noise to the picture, etc to see how your  algorithm behave with edge case (people in work package 3 also need a similar too, 

# Work package 3) -> Paul-Antoine
the goal is to compute the path planning using the current position + goal position and the matrix of occupancy and return a serie of waypoint for the robot to follow

ensure that the bounding box of the robot do not collide when following the waypoint.

```
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
```

To demonstrate how your system work, use the visualisation tool that render both the waypoint and the numpy array made by work package 4. try to pregen a few true/false map to see how your algorithm behave with edge case (no path exist, multiple path of same lenght exist, very short path, very large map and or Long path )


# Work package 4) -> Nathann
Slow loop and sensor fusion/kalman filter

the goal is to use all the function made by others and 
write sensor fusion for the position estimation.
write a visualtion tool to help the debugging and validation of WP 2 + 3

```

class PositionEstimator:
    def __init__(self):
        # Initialize Kalman filter matrices and parameters, keep it in memory

    def get_estimated_position(self, encoder_position, encoder_speed, camera_position):
        # Return refined position estimate
        return estimated_position

```

