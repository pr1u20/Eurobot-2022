from cv2 import aruco
import os

# Side length of the ArUco marker in meters 
aruco_marker_side_length = 0.05
aruco_ID42_side_lenght = 0.05

# CHARUCO CALIBRATION CONSTANTS
# the following call gets a ChArUco board of tiles 6 wide X 8 tall
CHARUCO_SQUARES_VERTICALLY = 6    # 180 < 210 mm
CHARUCO_SQUARES_HORIZONTALLY = 8  # 240 < 297 mm

CHARUCO_squareLength=0.03
CHARUCO_markerLength=0.015

CHARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

CALIBRATION_FILE_PATH = os.path.join("Vision_System", "calibration", "CameraCalibration.pckl")

