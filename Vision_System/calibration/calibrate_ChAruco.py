# The following code is used to watch a video stream, detect Aruco markers, and use
# a set of markers to determine the posture of the camera in relation to the plane
# of markers.

import numpy
import os
import cv2
from cv2 import aruco
import pickle
import Vision_System.parameters as param


gridboard = aruco.CharucoBoard(
        (param.CHARUCO_SQUARES_VERTICALLY, param.CHARUCO_SQUARES_VERTICALLY),  #SquareX, SquareY
        squareLength= param.CHARUCO_squareLength, 
        markerLength=param.CHARUCO_markerLength, 
        dictionary=param.CHARUCO_DICT)

# Corners discovered in all images processed
corners_all = []

# Aruco ids corresponding to corners discovered 
ids_all = [] 

# Determined at runtime
image_size = None 

# This requires a video taken with the camera you want to calibrate
file_path = param.CALIBRATION_VIDEO_PATH

if os.path.exists(file_path):
    # File exists, do something
    print(f"The file {file_path} exists.")
    vid = cv2.VideoCapture(file_path)

else:
    # File doesn't exist, do something else
    print(f"The file {file_path} does not exist. Start video with laptop camera.")
    vid = cv2.VideoCapture(0)

    
vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

# The more valid captures, the better the calibration
validCaptures = 0

frame_width = 1600
frame_height = 920

size = (frame_width, frame_height) 

# Below VideoWriter object will create 
# a frame of above defined The output  
# is stored in 'filename.avi' file. 
result = cv2.VideoWriter(param.SAVE_PROCESSED_CALIBRATION_VIDEO,  
                        cv2.VideoWriter_fourcc(*'MP4V'), 
                        30, size) 

# Loop through frames
while vid.isOpened():
    # Get frame
    ret, img = vid.read()

    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame or video finished. Exiting ...")
        break

    # Grayscale the image
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find aruco markers in the query image
    corners, ids, _ = aruco.detectMarkers(image=img, dictionary=param.CHARUCO_DICT)
    
    if len(corners) != 0:
        # Get charuco corners and ids from detected aruco markers
        response, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(markerCorners=corners,
                                                                                 markerIds=ids,
                                                                                 image=img,
                                                                                 board=gridboard)

        # Outline the aruco markers found in our query image
        img = aruco.drawDetectedMarkers(image=img, 
                                        corners=corners)

        # If a Charuco board was found, collect image/corner points
        # Requires at least 20 squares for a valid calibration image
        number_markers = int((param.CHARUCO_SQUARES_HORIZONTALLY*param.CHARUCO_SQUARES_VERTICALLY) / 2) - 1  # Number of Aruco markers in gridboard

        if response == number_markers:
            # Add these corners and ids to our calibration arrays
            corners_all.append(charuco_corners)
            ids_all.append(charuco_ids)
            
            # Draw the Charuco board we've detected to show our calibrator the board was properly detected
            img = aruco.drawDetectedCornersCharuco(image=img,
                                                   charucoCorners=charuco_corners,
                                                   charucoIds=charuco_ids)
        
            # If our image size is unknown, set it now
            if not image_size:
                image_size = img.shape[::-1]
            
            # Reproportion the image, maxing width or height at 1000
            proportion = max(img.shape) / 1000.0
            img = cv2.resize(img, (int(img.shape[1]/proportion), int(img.shape[0]/proportion)))

            validCaptures += 1
            if validCaptures == 1000:
                break

    # Pause to display each image, waiting for key press
    #img = cv2.resize(img, (1600, 920)) 
    result.write(img)
    cv2.imshow('Charuco board', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
          
# After the loop release the cap object
vid.release()
result.release()
# Destroy any open CV windows
cv2.destroyAllWindows()

# Show number of valid captures
print("{} valid captures".format(validCaptures))

if validCaptures < 40:
	print("Calibration was unsuccessful. We couldn't detect enough charucoboards in the video.")
	print("Perform a better capture or reduce the minimum number of valid captures required.")
	exit()

# Make sure we were able to calibrate on at least one charucoboard
if len(corners_all) == 0:
	print("Calibration was unsuccessful. We couldn't detect charucoboards in the video.")
	print("Make sure that the calibration pattern is the same as the one we are looking for (ARUCO_DICT).")
	exit()
print("Generating calibration...")

# Now that we've seen all of our images, perform the camera calibration
calibration, cameraMatrix, distCoeffs, rvecs, tvecs = aruco.calibrateCameraCharuco(
	charucoCorners=corners_all,
	charucoIds=ids_all,
	board=gridboard,
	imageSize=image_size,
	cameraMatrix=None,
	distCoeffs=None)
		
# Print matrix and distortion coefficient to the console
print("Camera intrinsic parameters matrix:\n{}".format(cameraMatrix))
print("\nCamera distortion coefficients:\n{}".format(distCoeffs))
		
# Save the calibration
f = open(param.CALIBRATION_FILE_PATH, 'wb')
pickle.dump((cameraMatrix, distCoeffs, rvecs, tvecs), f)
f.close()
		
# Print to console our success
print('Calibration successful. Calibration file created: {}'.format('CameraCalibration.pckl'))