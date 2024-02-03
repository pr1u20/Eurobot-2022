from cv2 import aruco
import os

# Side length of the ArUco marker in meters 
aruco_marker_side_length = 0.0452
aruco_ID42_side_lenght = 0.0455

# CHARUCO CALIBRATION CONSTANTS
# the following call gets a ChArUco board of tiles 6 wide X 8 tall
CHARUCO_SQUARES_VERTICALLY = 6    # 180 < 210 mm
CHARUCO_SQUARES_HORIZONTALLY = 8  # 240 < 297 mm

CHARUCO_squareLength=0.030
CHARUCO_markerLength=0.015

CHARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

CALIBRATION_FILE_PATH = os.path.join("Vision_System", "calibration", "CameraCalibration.pckl")
CALIBRATION_VIDEO_PATH = os.path.join("Vision_System", "media", "Calibration_2.mp4")
VIDEO_PATH = os.path.join("Vision_System", "media", "video_close_2.mp4")

