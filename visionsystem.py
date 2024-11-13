import numpy as np
import cv2

class VisionSystem:
    def __init__(self):
        return None# Initialize camera and other parameters

    def is_camera_ready(self):
        # Check if the camera is ready (returns True or False)
        cap = cv2.VideoCapture(0)

        if cap.isOpened():
            return True
        else:
            return False # if the camera is not ready

    def capture_frame(self):
        if not self.is_camera_ready():
            return None

        cap = cv2.VideoCapture(0)
        capture, frame = cap.read() #capture = bouleen and frame = (hauteur, largeur, nbr_colorRGB_use)

        if capture:
            cap.release()
            return frame
        else:
            cap.release()
            return None

    def get_robot_position(self):
        # Use OpenCV to detect robot’s position and orientation
        # Example position and orientation
        return np.array([100, 150, np.pi / 6])  # x=100mm, y=150mm, direction=30° (π/6 radians)

    def get_goal_position(self):
        # Detect and return the goal position
        # The positioning is absolute and explained in generate_occupancy_grid
        return np.array([100, 150, np.pi / 6])  # x=100mm, y=150mm, direction=30° (π/6 radians)

    def generate_occupancy_grid(self):
        # Create and return an NxM occupancy grid based on the map layout
        # True means the space is occupied, False means it's free
        # Grid layout: 0,0 is the lower left corner, rows are along x and columns along y

        matrix = []
        for i in range(120):  # Set grid dimensions as required
            row = []
            for j in range(150):
                row.append(True if np.random.rand() > 0.5 else False)
            matrix.append(row)

        return np.array(matrix)  # Convert the list of lists into a NumPy array


def display_first_five_aruco_markers():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

    for i in range(5):  # Affiche les 5 premiers marqueurs de 0 à 4
        marker_image = cv2.aruco.generateImageMarker(aruco_dict, i, 200)

        cv2.imwrite(f'result\Marqueur {i}.png', marker_image)



def visionmain():

    print("create vision object")
    visionsystem = VisionSystem()
    print("take a picture")
    frame = visionsystem.capture_frame()
    print("save picture")
    cv2.imwrite('result\captured_frame.png', frame)  # Save to file result



if __name__ == "__main__":
    #visionmain()
    display_first_five_aruco_markers()
