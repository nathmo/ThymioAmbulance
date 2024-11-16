import numpy as np
import cv2
from helper import *

class VisionSystem:
    def __init__(self):
        # Charger le dictionnaire de marqueurs (DICT_6X6_250 contient 250 marqueurs uniques)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        # Paramètres de la caméra (à remplacer par les valeurs de calibration si disponibles)
        self.camera_matrix = np.array([[800, 0, 320], [0, 800, 240], [0, 0, 1]], dtype=float)
        self.dist_coeffs = np.array([0, 0, 0, 0], dtype=float)

        # Longueur du côté du marqueur en mètres
        self.marker_length = 0.05

        self.use_camera = 0  # Booléen true si on utilise camera, false si on veut utiliser image
        self.image_path = "testData\Image_iphone_path.png" # Chemin de l'image si use_camera est False

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

    def get_frame(self):
        #Function qui determine si on prend camera ou frame choisi
        if self.use_camera:
            print("Utilisation de la caméra")
            frame = self.capture_frame()
            if frame is None:
                print("Erreur lors de la capture de la frame depuis la caméra.")
            return frame
        else:
            print(f"Chargement de l'image depuis {self.image_path}")
            frame = cv2.imread(self.image_path)
            if frame is None:
                print(f"Erreur : Impossible de charger l'image depuis {self.image_path}.")
            return frame

    def get_robot_position(self):
        # Use OpenCV to detect robot’s position and orientation
        # Example position and orientation
        # return np.array([100, 150, np.pi / 6])  # x=100mm, y=150mm, direction=30° (π/6 radians)

        frame = self.get_frame() #prendre image qu'on veut camera ou image
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        print(ids)
        if ids is not None:
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix,
                                                                  self.dist_coeffs)

            robot_position = None

            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 5: #check si on a bien 4 marqueurs
                    x, y = tvecs[i][0][:2] * 1000 #[:2] nous permet de prendre que x et y sans z

                    rotation_matrix, _ = cv2.Rodrigues(rvecs[i][0])
                    orientation = np.arctan2(rotation_matrix[1, 0],rotation_matrix[0, 0])  #Angle en radians XY

                    # Stocker la position et l'orientation du robot
                    robot_position = np.array([x, y, orientation])

            if robot_position is not None:
                return robot_position
            else:
                print("Marqueur du robot non détecté.")
                return None
        else:
            print("Aucun marqueur Robot détecté.")
            return None


    def get_goal_position(self):
        # Detect and return the goal position
        # The positioning is absolute and explained in generate_occupancy_grid
        # return np.array([100, 150, np.pi / 6])  # x=100mm, y=150mm, direction=30° (π/6 radians)

        frame = self.get_frame() #prendre image qu'on veut camera ou image
        corners, ids, _ = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)

        if ids is not None:
            ids = ids.flatten()

            # Vérification de la présence des marqueurs nécessaires (0, 1, 2, 3 pour les coins et 5 pour l'objectif)
            required_markers = {0, 1, 2, 3, 5}
            detected_markers = set(ids)

            if not required_markers.issubset(detected_markers):
                print("Tous les marqueurs dans goal nécessaires ne sont pas détectés.")
                print()
                return None

            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix,
                                                                  self.dist_coeffs)

            #Position de l'objectif (marqueur 5)
            for i, marker_id in enumerate(ids):
                if marker_id == 5:
                    x, y = tvecs[i][0][:2] * 1000  # [:2] permet de prendre uniquement x et y sans z
                    orientation = 0  # Utilisation d'une valeur fixe pour l'orientation
                    return np.array([x, y, orientation])

        else:
            print("Aucun marqueur goal détecté.")
            return None


    def generate_occupancy_grid(self):
        # Create and return an NxM occupancy grid based on the map layout
        # True means the space is occupied, False means it's free
        # Grid layout: 0,0 is the lower left corner, rows are along x and columns along y

        frame = self.get_frame() #prendre image qu'on veut camera ou image

        # Étape 1: Convertir l'image en niveaux de gris
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Étape 2: Appliquer un filtre pour accentuer les objets, applique filtre pour réduire le bruit
        blurred_frame = cv2.GaussianBlur(gray_frame, (5, 5), 0)

        # Étape 3: Appliquer un seuil (Otsu) pour binariser l'image
        _, binary_frame = cv2.threshold(blurred_frame, 0, 1, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

        return binary_frame
        '''
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()

        if ret:
            binary_matrix = frame_to_binary_matrix(frame)
            print(binary_matrix)  # Afficher la matrice binaire composée de 0 et 1
    
            # Afficher l'image binaire pour vérifier le résultat
            cv2.imshow('Binary Frame', binary_matrix * 255)
            cv2.waitKey(0)
    
        cap.release()
        cv2.destroyAllWindows()

    
        matrix = []
        for i in range(120):  # Set grid dimensions as required
            row = []
            for j in range(150):
                row.append(True if np.random.rand() > 0.5 else False)
            matrix.append(row)

        return np.array(matrix)  # Convert the list of lists into a NumPy array
    '''

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
    #display_first_five_aruco_markers()
    visionsystem = VisionSystem()
    occupancyGrid=visionsystem.generate_occupancy_grid()
    goal = visionsystem.get_goal_position()# slightly different than the default camera one
    robot = visionsystem.get_robot_position()  # slightly different than the default camera one
    print(robot)
    print(goal)
    robotSpeedFromEncoder = np.array([0, 0, 0]) #no speed
    waypoints = [
        np.array([100, 150, np.pi / 6]),  # x=100mm, y=150mm, direction=30° (π/6 radians)
        np.array([200, 250, np.pi / 3]),  # x=200mm, y=250mm, direction=60° (π/3 radians)
        np.array([300, 350, np.pi / 2])  # x=300mm, y=350mm, direction=90° (π/2 radians)
    ]
    plot_robot_grid(occupancyGrid, 10, robot, robot, robot, goal, waypoints)
