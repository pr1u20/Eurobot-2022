from cv2 import aruco
import os

# Side length of the ArUco marker in meters 
aruco_marker_side_length = 0.0452
aruco_ID42_side_lenght = 0.0455

# CHARUCO CALIBRATION CONSTANTS
# the following call gets a ChArUco board of tiles 6 wide X 8 tall
CHARUCO_SQUARES_VERTICALLY = 5    # 180 < 210 mm or 250 < 297 mm
CHARUCO_SQUARES_HORIZONTALLY = 7  # 240 < 297 mm or 385 < 420 mm

CHARUCO_squareLength=0.055
CHARUCO_markerLength=0.03

CHARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

CALIBRATION_FILE_PATH = os.path.join("Vision_System", "calibration", "CameraCalibration.pckl")
CALIBRATION_VIDEO_PATH = os.path.join("Vision_System", "media", "Calibration_2.mp4")
VIDEO_PATH = os.path.join("Vision_System", "media", "video_close_2.mp4")

