import numpy as np
import cv2

class VisionSystem:
    def __init__(self):
        # Charger le dictionnaire de marqueurs (DICT_6X6_250 contient 250 marqueurs uniques)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Paramètres de la caméra (à remplacer par les valeurs de calibration si disponibles)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.array([0, 0, 0, 0], dtype=float)

        # Longueur du côté du marqueur en mètres
        self.marker_length = 0.05

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
        # return np.array([100, 150, np.pi / 6])  # x=100mm, y=150mm, direction=30° (π/6 radians)

        frame = self.capture_frame()
        if frame is None:
            print("Erreur lors de la capture de la frame.")
            return None

        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix,
                                                                  self.dist_coeffs)

            robot_position = None

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 5: #check si on a bien 4 marqueurs
                    x, y = tvecs[i][0][:2] * 1000 #[:2] nous permet de prendre que x et y sans z

                    # Utilisation d'une approche simplifiée pour l'orientation
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                    orientation = np.arctan2(rotation_matrix[1, 0],rotation_matrix[0, 0])  #Angle en radians XY

                    # Stocker la position et l'orientation du robot
                    robot_position = np.array([x, y, orientation])

            # Retourner la position et l'orientation du robot s'il est détecté
            if robot_position is not None:
                return robot_position
            else:
                print("Marqueur du robot non détecté.")
                return None
        else:
            print("Aucun marqueur détecté.")
            return None


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

    for i in range(6):  # Affiche les 5 premiers marqueurs de 0 à 5
        marker_image = cv2.aruco.generateImageMarker(aruco_dict, i, 200)

        cv2.imwrite(f'Marqueur {i}.png', marker_image)



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
