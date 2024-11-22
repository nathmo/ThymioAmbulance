import numpy as np
import cv2
from helper import *

class VisionSystem:
    def __init__(self, use_camera=False, image_path="testData\Image_iphone_path.png", checker_board_length=175):
        # Charger le dictionnaire de marqueurs (DICT_6X6_250 contient 250 marqueurs uniques)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # Paramètres de la caméra (à remplacer par les valeurs de calibration si disponibles)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.array([0, 0, 0, 0], dtype=float)

        # Longueur du côté du marqueur en mètres
        self.pixel_side_dimmension_mm = 1 # [mm]
        self.checker_board_length = checker_board_length # [mm]

        self.use_camera = use_camera  # Booléen true si on utilise camera, false si on veut utiliser une image statique
        self.image_path = image_path # Chemin de l'image si use_camera est False

        self.aruco_robot_id = 5
        self.aruco_target_id = 4

        #Function qui determine si on prend camera ou frame choisi
        if self.use_camera:
            print("Utilisation de la caméra")
        else:
            print(f"Chargement de l'image depuis {self.image_path}")

        # Will calibrate the camera by finding the checkboard pattern. Can be called
        # Later if the calibration failed.
        self.calibrate_camera_with_checkerboard()

        return None

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

    def get_frame(self):
        #Function qui determine si on prend camera ou frame choisi
        if self.use_camera:
            frame = self.capture_frame()
            if frame is None:
                print("Erreur lors de la capture de la frame depuis la caméra.")
            return frame
        else:
            frame = cv2.imread(self.image_path)
            if frame is None:
                print(f"Erreur : Impossible de charger l'image depuis {self.image_path}.")
            return frame

    def get_pixel_side_mm(self):
        return self.pixel_side_dimmension_mm

    def calibrate_camera_with_checkerboard(self):
        """
        Calibrates the camera using an 8x8 checkerboard pattern and computes the size of a pixel in millimeters.
        Updates self.camera_matrix and self.dist_coeffs.

        Returns:
            None
        """

        frame = self.get_frame()

        # Checkerboard dimensions (inner corners: one less than the squares in each row/col)
        checkerboard_dims = (7, 7)  # 8x8 checkerboard has 7x7 inner corners

        # Prepare object points (3D points in real-world space)
        square_size = self.checker_board_length / 8  # Size of one square in millimeters
        objp = np.zeros((checkerboard_dims[0] * checkerboard_dims[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:checkerboard_dims[0], 0:checkerboard_dims[1]].T.reshape(-1, 2)
        objp *= square_size  # Scale according to real-world square size

        # Find the checkerboard corners in the image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, checkerboard_dims, None)

        if ret:  # If corners are found
            print("Checkerboard detected. Performing camera calibration...")

            # Refine corner positions for better accuracy
            corners_refined = cv2.cornerSubPix(
                gray,
                corners,
                winSize=(11, 11),
                zeroZone=(-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )

            # Perform camera calibration
            ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                [objp],  # Object points
                [corners_refined],  # Image points
                gray.shape[::-1],  # Image size (width, height)
                None,  # Initial camera matrix
                None  # Initial distortion coefficients
            )

            if ret:  # If calibration is successful
                # Update the object's camera matrix and distortion coefficients
                self.camera_matrix = camera_matrix
                self.dist_coeffs = np.array([0, 0, 0, 0], dtype=float)  # As per request

                # Compute pixel size by directly measuring the checkerboard's sides
                top_left = corners_refined[0][0]  # Top-left corner
                top_right = corners_refined[6][0]  # Top-right corner
                bottom_left = corners_refined[42][0]  # Bottom-left corner
                bottom_right = corners_refined[48][0]  # Bottom-right corner

                # Measure the horizontal and vertical pixel distances
                horizontal_pixel_distance = np.linalg.norm(top_right - top_left)
                vertical_pixel_distance = np.linalg.norm(bottom_left - top_left)

                # Compute pixel size in millimeters
                pixel_size_mm_horizontal = self.checker_board_length / horizontal_pixel_distance
                pixel_size_mm_vertical = self.checker_board_length / vertical_pixel_distance

                # Average pixel size
                self.pixel_side_dimmension_mm = (pixel_size_mm_horizontal + pixel_size_mm_vertical) / 2

                print(f"Calibration successful.")
                print(f"Horizontal pixel size: {pixel_size_mm_horizontal} mm")
                print(f"Vertical pixel size: {pixel_size_mm_vertical} mm")
                print(f"Averaged pixel size: {self.pixel_side_dimmension_mm} mm")
            else:
                print("Calibration failed.")
        else:
            print("Checkerboard not detected in the frame.")

        return None

    def get_robot_position(self):
        """
        Detect ArUco markers, compute the robot's position and orientation,
        and calculate the angle between the x-axis and the marker.
        """

        frame = self.get_frame()

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        # If markers are detected, proceed
        if ids is not None:
            # List to store marker centers and their respective IDs
            marker_centers = []

            for i, corner in enumerate(corners):
                # Compute the pixel position of the marker's center
                center_x = int((corner[0][0][0] + corner[0][2][0]) / 2) #1er index choose which Aruco,
                center_y = int((corner[0][0][1] + corner[0][2][1]) / 2) #2eme give which corner, 3eme tell us x or y
                marker_centers.append((ids[i][0], center_x, center_y))

            # Find marker ID 5 (robot marker)
            marker_robot = next((m for m in marker_centers if m[0] == self.aruco_robot_id), None)

            if marker_robot:
                # Get the corners of marker ID 5 to compute orientation
                idx = list(ids.flatten()).index(5)  # Index of marker ID 5
                corners_robot = corners[idx][0]  # Extract corners of marker ID 5

                # Define two adjacent corners for angle calculation
                corner1 = corners_robot[0]  # Top-left corner
                corner2 = corners_robot[1]  # Top-right corner

                # Compute vector along the marker's x-axis in pixel space
                vector_x = corner2 - corner1  # Vector from corner1 to corner2
                angle_radians = np.arctan2(vector_x[1], vector_x[0])  # Angle with x-axis

                # Ensure clockwise convention: invert y-axis (screen coordinates)
                angle_radians = -angle_radians

                # Convert the robot's pixel position to mm
                position_x_mm = float(marker_robot[1]) * self.pixel_side_dimmension_mm
                position_y_mm = float(marker_robot[2]) * self.pixel_side_dimmension_mm

                return np.array([position_x_mm, position_y_mm, angle_radians])

            else:
                print("Marker ID for robot not found in the frame.")
        else:
            print("No ArUco markers detected.")
        # Returning an example value as placeholder
        return None


    def get_goal_position(self):
        # Detect and return the goal position
        # The positioning is absolute and explained in generate_occupancy_grid
        # return np.array([100, 150, np.pi / 6])  # x=100mm, y=150mm, direction=30° (π/6 radians)

        frame = self.get_frame()

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        # If markers are detected, proceed
        if ids is not None:
            # List to store marker centers and their respective IDs
            marker_centers = []

            for i, corner in enumerate(corners):
                # Compute the pixel position of the marker's center
                center_x = int((corner[0][0][0] + corner[0][2][0]) / 2)
                center_y = int((corner[0][0][1] + corner[0][2][1]) / 2)
                marker_centers.append((ids[i][0], center_x, center_y))

            # Find marker ID 5 (robot marker)
            marker = next((m for m in marker_centers if m[0] == self.aruco_target_id), None)

            if marker:
                # Get the corners of marker ID 5 to compute orientation
                idx = list(ids.flatten()).index(5)  # Index of marker ID 5
                corners_robot = corners[idx][0]  # Extract corners of marker ID 5

                # Define two adjacent corners for angle calculation
                corner1 = corners_robot[0]  # Top-left corner
                corner2 = corners_robot[1]  # Top-right corner

                # Compute vector along the marker's x-axis in pixel space
                vector_x = corner2 - corner1  # Vector from corner1 to corner2
                angle_radians = np.arctan2(vector_x[1], vector_x[0])  # Angle with x-axis

                # Ensure clockwise convention: invert y-axis (screen coordinates)
                angle_radians = -angle_radians

                # Convert the robot's pixel position to mm
                position_x_mm = float(marker[1]) * self.pixel_side_dimmension_mm
                position_y_mm = float(marker[2]) * self.pixel_side_dimmension_mm

                return np.array([position_x_mm, position_y_mm, angle_radians])

            else:
                print("Marker ID for goal not found in the frame.")
        else:
            print("No ArUco markers detected.")
        # Returning an example value as placeholder
        return None


    def generate_occupancy_grid(self):
        # Create and return an NxM occupancy grid based on the map layout
        # True means the space is occupied, False means it's free
        # Grid layout: 0,0 is the lower left corner, rows are along x and columns along y

        frame = self.get_frame()

        # Étape 1: Convertir l'image en niveaux de gris
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Étape 2: Appliquer un filtre pour accentuer les objets, applique filtre pour réduire le bruit
        blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

        # Étape 3: Appliquer un seuil (Otsu) pour binariser l'image
        _, binary_frame = cv2.threshold(blurred_frame, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        return binary_frame

    def display_first_five_aruco_markers():
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

        for i in range(6):  # Affiche les 5 premiers marqueurs de 0 à 5
            marker_image = cv2.aruco.generateImageMarker(aruco_dict, i, 200)

            cv2.imwrite(f'Marqueur {i}.png', marker_image)



if __name__ == "__main__":
    visionsystem = VisionSystem()
    occupancyGrid = visionsystem.generate_occupancy_grid()
    goal = visionsystem.get_goal_position() # slightly different than the default camera one
    robot = visionsystem.get_robot_position()  # slightly different than the default camera one
    print("position robot :")
    print(robot)
    print("position goal :")
    print(goal)
    robotSpeedFromEncoder = np.array([0, 0, 0]) #no speed
    waypoints = [
        np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians)
    ]
    plot_robot_grid(occupancyGrid, visionsystem.get_pixel_side_mm(), robot, robot, robot, goal, waypoints)
