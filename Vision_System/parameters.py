from cv2 import aruco
import os

# Side length of the ArUco marker in meters 
aruco_marker_side_length = 0.230
aruco_ID42_side_lenght = 0.230

# CHARUCO CALIBRATION CONSTANTS
# the following call gets a ChArUco board of tiles 6 wide X 8 tall
CHARUCO_SQUARES_VERTICALLY = 5    # 180 < 210 mm or 250 < 297 mm
CHARUCO_SQUARES_HORIZONTALLY = 7  # 240 < 297 mm or 385 < 420 mm

CHARUCO_squareLength=0.0551
CHARUCO_markerLength=0.0301

CHARUCO_DICT = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)

CALIBRATION_FILE_PATH = os.path.join("Vision_System", "calibration", "CameraCalibration.pckl")
CALIBRATION_VIDEO_PATH = os.path.join("Vision_System", "ARUCO testing Surrey", "calibration_1.mp4")

filename = "steady_right_top_edited_1"
VIDEO_PATH = os.path.join("Vision_System", "ARUCO testing Surrey", f"{filename}.mp4")
PATH_TO_SAVED_DATA = os.path.join("Vision_System", "data", f"{filename}.csv")

SAVE_PROCESSED_VIDEO = os.path.join("Vision_System", "data", f"{filename}.mp4")
SAVE_PROCESSED_CALIBRATION_VIDEO = os.path.join("Vision_System", "media", "processed_calibration_1.mp4")

