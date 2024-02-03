import cv2
import cv2.aruco as aruco
import os
import Vision_System.parameters as param

# Create ChArUco board, which is a set of Aruco markers in a chessboard setting
# meant for calibration
# the following call gets a ChArUco board of tiles 5 wide X 7 tall
SQUARES_VERTICALLY = param.CHARUCO_SQUARES_VERTICALLY    # 180 < 210 mm
SQUARES_HORIZONTALLY = param.CHARUCO_SQUARES_VERTICALLY  # 240 < 297 mm

    
gridboard = aruco.CharucoBoard(
        (param.CHARUCO_SQUARES_VERTICALLY, param.CHARUCO_SQUARES_VERTICALLY),  #SquareX, SquareY
        squareLength= param.CHARUCO_squareLength, 
        markerLength=param.CHARUCO_markerLength, 
        dictionary=param.CHARUCO_DICT)

# Create an image from the gridboard
size_ratio = SQUARES_HORIZONTALLY / SQUARES_VERTICALLY
board_image = gridboard.generateImage((1000, int(1000*size_ratio)), None, 0, 1)

path = os.path.join("Vision_System", "calibration", "test_charuco.jpg")
cv2.imwrite(path, board_image)

# Display the image to us
cv2.imshow('Gridboard', board_image)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()