import cv2
import cv2.aruco as aruco

# Create ChArUco board, which is a set of Aruco markers in a chessboard setting
# meant for calibration
# the following call gets a ChArUco board of tiles 5 wide X 7 tall
SQUARES_VERTICALLY = 6    # 180 < 210 mm
SQUARES_HORIZONTALLY = 8  # 240 < 297 mm

    
gridboard = aruco.CharucoBoard(
        (SQUARES_VERTICALLY, SQUARES_HORIZONTALLY),  #SquareX, SquareY
        squareLength=0.03, 
        markerLength=0.015, 
        dictionary=aruco.getPredefinedDictionary(aruco.DICT_5X5_50))

# Create an image from the gridboard
size_ratio = SQUARES_HORIZONTALLY / SQUARES_VERTICALLY
board_image = gridboard.generateImage((1000, int(1000*size_ratio)), None, 0, 1)

cv2.imwrite("Vision System/calibration/test_charuco.jpg", board_image)

# Display the image to us
cv2.imshow('Gridboard', board_image)
# Exit on any key
cv2.waitKey(0)
cv2.destroyAllWindows()