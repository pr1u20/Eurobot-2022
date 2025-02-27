#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Feb 16 20:54:11 2022

@author: user
"""

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
import math
import pickle
import os
import matplotlib.pyplot as plt
import time
import pandas as pd

import Vision_System.parameters as param


width = 1500
height = 900

# Camera matrix
mtx = np.array([[800, 0., width / 2],
                    [0., 800, height / 2],
                    [0., 0., 1.]])
            
# Distortion parameters
dst = np.array([1.0557813457472758e-01, 1.0526580754799850e+00,
           4.6737646216940517e-03, 2.9071112143870346e-03,
           -6.2540648080086747e+00])

dst = np.array([0.001, 0.001, 0.001, 0.001, 0.001])

# Calibration parameters yaml file
camera_calibration_parameters_filename = 'calibration_chessboard.yaml'
 
# Load the camera parameters from the saved file
f = open(param.CALIBRATION_FILE_PATH, 'rb')
(mtx, dst, _, _) = pickle.load(f)
f.close()


def euler_from_quaternion(x, y, z, w):
  """
  Convert a quaternion into euler angles (roll, pitch, yaw)
  roll is rotation around x in radians (counterclockwise)
  pitch is rotation around y in radians (counterclockwise)
  yaw is rotation around z in radians (counterclockwise)
  """
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll_x = math.atan2(t0, t1)
      
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch_y = math.asin(t2)
      
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw_z = math.atan2(t3, t4)
  
  return roll_x, pitch_y, yaw_z # in radians

def get_id42_coordinates():
    # Split the video path by underscores
    path_parts = param.VIDEO_PATH.split('\\')

    name = path_parts[-1]

    path_parts = name.split('_')

    # Extract the "left" and "bottom" strings
    left_or_right_string = path_parts[1]
    bottom_or_top_string = path_parts[2]

    print(left_or_right_string)

    if left_or_right_string == "left":
        x = 0

    elif left_or_right_string == "right":
        x = 2000

    if bottom_or_top_string == "bottom":

        y = 0
    
    elif bottom_or_top_string == "top":

        y = 3000

    return x, y


class Aruco():
    """
    Object used to process the ArUco tags.
    """
    
    def __init__(self, mtx, dst, aruco_dict_type, length_marker = 0.02):
        self.mtx = mtx
        self.dst = dst
        self.arucoDict = cv2.aruco.getPredefinedDictionary(aruco_dict_type)
        self.arucoParams = cv2.aruco.DetectorParameters()
        # Set parameters as needed
        #self.arucoParams.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_CONTOUR
        #self.arucoParams.cornerRefinementMinAccuracy = 2
        self.arucoParams.adaptiveThreshWinSizeMin = 3
        self.arucoParams.adaptiveThreshWinSizeMax = 40
        self.arucoParams.adaptiveThreshWinSizeStep = 2
        self.arucoParams.adaptiveThreshConstant = 20
        self.length_marker = length_marker
        x, y = get_id42_coordinates()
        self.X_42 = x  #X position of tag with ID 42, our reference tag.
        self.Y_42 = y #Y position of tag with ID 42, our reference tag.
        self.RA_ID = 4    # Id of robot A
        self.RB_ID = 12   # Id of robot B
        self.poses = []
        fps = 30 # 
        self.dt = 1 / fps
        
    def detection(self, frame):
        
        """
        Detect and draw the corners and id  of the ArUco markers.
        """
        
        self.frame = frame
    
        self.corners, self.marker_ids, rejected_img_points = cv2.aruco.detectMarkers(self.frame, 
                                                                                     self.arucoDict, 
                                                                                     parameters=self.arucoParams)
        
        # Draw a square around the markers
        cv2.aruco.drawDetectedMarkers(self.frame, self.corners, self.marker_ids)
        
        self.marker_ids = np.squeeze(self.marker_ids)
        
        try:
            if self.marker_ids == None: self.marker_ids = None
        
        except ValueError as r:
            print(r)
            
        try:
            if not isinstance(int(self.marker_ids), type(int)):
                self.marker_ids = [int(self.marker_ids)]
            
        except TypeError:
            pass
        
        return self.corners, self.marker_ids
        
        

    def pose_estimation(self, frame, draw = False):
    
        '''
        Get the pose (position and orientation and draw on image).
        frame - Frame from the video stream
        matrix_coefficients - Intrinsic matrix of the calibrated camera
        distortion_coefficients - Distortion coefficients associated with your camera
        return:-
        frame - The frame with the axis drawn on it
        '''
        self.frame = frame
    
        self.detection(self.frame)
        
    
        # Check that at least one ArUco marker was detected
        if not isinstance(self.marker_ids, type(None)):
            print(self.marker_ids)

            self.rotation_list = []
            self.position_list = []
            # Print the pose for the ArUco marker
            # The pose of the marker is with respect to the camera lens frame.
            # Imagine you are looking through the camera viewfinder, 
            # the camera lens frame's:
            # x-axis points to the right
            # y-axis points straight down towards your toes
            # z-axis points straight ahead away from your eye, out of the camera

            for i, marker_id in enumerate(self.marker_ids):
                
                
                if marker_id == 42:
                    lenght = param.aruco_ID42_side_lenght
                else: lenght = self.length_marker
                # Get the rotation and translation vectors
                self.rvecs, self.tvecs, self.obj_points = cv2.aruco.estimatePoseSingleMarkers(
                                             self.corners[i],
                                             lenght,
                                             self.mtx,
                                             self.dst)
                
                self.position_list.append(self.tvecs[0][0])
    
                if draw == True: cv2.drawFrameAxes(self.frame, self.mtx, self.dst, self.rvecs[0][0], self.tvecs[0][0], self.length_marker)
                
                # Store the translation (i.e. position) information
                transform_translation_x = self.tvecs[0][0][0]
                transform_translation_y = self.tvecs[0][0][1]
                transform_translation_z = self.tvecs[0][0][2]
                 
                # Store the rotation information
                rotation_matrix = np.eye(4)
                rotation_matrix[0:3, 0:3] = cv2.Rodrigues(np.array(self.rvecs[0][0]))[0]
                r = R.from_matrix(rotation_matrix[0:3, 0:3])
                quat = r.as_quat()   
                 
                # Quaternion format     
                transform_rotation_x = quat[0] 
                transform_rotation_y = quat[1] 
                transform_rotation_z = quat[2] 
                transform_rotation_w = quat[3] 
                 
                # Euler angle format in radians
                roll_x, pitch_y, yaw_z = euler_from_quaternion(transform_rotation_x, 
                                                               transform_rotation_y, 
                                                               transform_rotation_z, 
                                                               transform_rotation_w)
                 
                
                self.rotation_list.append([roll_x, pitch_y, yaw_z])
                
                roll_x = math.degrees(roll_x)
                pitch_y = math.degrees(pitch_y)
                yaw_z = math.degrees(yaw_z)
                #print("transform_translation_x: {}".format(transform_translation_x))
                #print("transform_translation_y: {}".format(transform_translation_y))
                #print("transform_translation_z: {}".format(transform_translation_z))
                #print("roll_x: {}".format(roll_x))
                #print("pitch_y: {}".format(pitch_y))
                #print("yaw_z: {}".format(yaw_z))
                #print()
                        
            self.rotation_list = np.array(self.rotation_list)
            self.position_list = np.array(self.position_list)

        return self.frame
    
    def transformationEuler(self, id1, id2, id_to_arg):
        
        """Rotate and translate the reference axis (origin) and get the 
        coordinates of the tags in terms of our reference tag axis. """
        
        if id_to_arg:
            arg_id1 = np.where(self.marker_ids == id1)[0][0]
            arg_id2 = np.where(self.marker_ids == id2)[0][0]
            
        else:
            arg_id1 = id1
            arg_id2 = id2
        
        P1 = self.position_list[arg_id1]
        P2 = self.position_list[arg_id2]
        
        # Store the rotation information
        r = R.from_euler("xyz", self.rotation_list[arg_id2])
        
        # Apply rotation to vector between two points
        P1_t = r.apply(P1 - P2, inverse = True)
        
        return P1_t    
    
    def relevantPose(self, id1, id2, id_to_arg = True):
        
        """Get the three relevant pose parameters: x position, y position and 
        angle of rotation (orientation)"""
        
        # If the ids given are not already arguments, convert to arguments
        if id_to_arg:
            arg_id1 = np.where(self.marker_ids == id1)[0][0]
            arg_id2 = np.where(self.marker_ids == id2)[0][0]
            
        else:
            arg_id1 = id1
            arg_id2 = id2
        
        distance_vector = self.transformationEuler(id1, id2, id_to_arg)
        
        X = distance_vector[0] * 1000
        Y = distance_vector[1] * 1000
        
        rotation_degrees = self.rotation_list * 180 / np.pi
        
        angle = rotation_degrees[arg_id2][-1] - rotation_degrees[arg_id1][-1]
        
        if angle > 180:
            angle = angle - 360
            
        elif angle < -180:
            angle = angle + 360
        
        return X + self.X_42, Y + self.Y_42, angle
    
    def respective_poses(self):
        
        """Get poses of all markers with respect to our reference marker"""
        
        # Reference marker
        id2 = 42
        arg_id2 = np.where(self.marker_ids == id2)[0][0]
        
        pose = []
        
        for arg_id1 in range(len(self.marker_ids)):
             
            X, Y, angle = self.relevantPose(arg_id1, arg_id2, id_to_arg = False)
            pose.append([X, Y, angle])
        
        pose = np.array(pose)
        return pose
    
    def pixel_distance(self, id1, id2):
        """Get the pixel distance between two markers"""
        
        arg_id1 = np.where(self.marker_ids == id1)
        arg_id2 = np.where(self.marker_ids == id2)
        
        pos_id1 = self.markers_position[arg_id1][0]
        pos_id2 = self.markers_position[arg_id2][0]
        
        pixel_dist = pos_id2 - pos_id1
        
        return pixel_dist
    
    def comparingLenght(self, id1, id2):
        
        """Make sure Euler transformation was correct and maginutde of 
        distance vecotr is the same before and after the transformation."""
        
        arg_id1 = np.where(self.marker_ids == id1)[0][0]
        arg_id2 = np.where(self.marker_ids == id2)[0][0]
        
        dist_1 = self.tvecs[arg_id1] - self.tvecs[arg_id2]
        dist_2 = self.transformationEuler(id1, id2)
        
        return np.sum(dist_1**2), np.sum(dist_2**2)
    
    
    def live_processing(self, image):
        
        image = self.pose_estimation(image, draw = True)
        AB_pose = np.zeros(6)  # Array to store pose of robot a and b

        pose_to_append = [np.nan, np.nan, np.nan] #in case of not detetected
        
        if not isinstance(self.marker_ids, type(None)) and 42 in self.marker_ids and len(self.marker_ids) > 1:
            poses = self.respective_poses()
            
            for i, pose in enumerate(poses):

                if self.marker_ids[i] == 6:
                    pose_to_append = pose

                print(f"X: {round(pose[0])} mm --- Y: {round(pose[1])} mm --- Angle: {round(pose[2], 2)}° --- ID: {self.marker_ids[i]}")
                if self.marker_ids[i] == self.RA_ID:
                    AB_pose[:3] = np.round(pose[:3])
                elif self.marker_ids[i] == self.RB_ID:
                    AB_pose[-3:] = np.round(pose[:3])
        else:
            print("No detected markers")
            poses = None

        self.poses.append(pose_to_append)
            
        return image, AB_pose
    
    def plot(self):

        t = np.linspace(0, len(self.poses)*self.dt, len(self.poses))
        self.poses = np.array(self.poses)

        df = pd.DataFrame()

        df['t (s)'] = t
        df['x (mm)'] = self.poses[:, 0]
        df['y (mm)'] = self.poses[:, 1]
        df['angle (°)'] = self.poses[:, 2]

        df.to_csv(param.PATH_TO_SAVED_DATA)

        plt.subplot(3, 2, 1)
        plt.plot(t, self.poses[:, 0])
        plt.title("x position")
        plt.xlabel("t (s)")
        plt.ylabel("position (m)")

        plt.subplot(3, 2, 2)
        plt.plot(t, self.poses[:, 1])
        plt.title("y position")
        plt.xlabel("t (s)")
        plt.ylabel("position (m)")


        plt.subplot(3, 2, 3)
        plt.plot(t, self.poses[:, 2])
        plt.title("z rotation (ψ)")
        plt.xlabel("t (s)")
        plt.ylabel("angle (°)")

        plt.tight_layout()
        plt.show()
        
def main(show = False):
    
    aruco = Aruco(mtx, dst, aruco_dict_type = cv2.aruco.DICT_4X4_100, length_marker = param.aruco_marker_side_length)
    
    file_path = param.VIDEO_PATH

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

    frame_width = 1600
    frame_height = 920

    size = (frame_width, frame_height) 
   
    # Below VideoWriter object will create 
    # a frame of above defined The output  
    # is stored in 'filename.avi' file. 
    result = cv2.VideoWriter(param.SAVE_PROCESSED_VIDEO,  
                            cv2.VideoWriter_fourcc(*'MP4V'), 
                            30, size) 
      
    while vid.isOpened():
          
        # Capture the video frame
        # by frame
        ret, frame = vid.read()

        # if frame is read correctly ret is True
        if not ret:
            print("Can't receive frame or video finished. Exiting ...")
            break
        
        #process image
        frame, _ = aruco.live_processing(frame)
      
        # Display the resulting frame
        
        if show:
            frame = cv2.resize(frame, (1600, 920)) 
            result.write(frame)
            cv2.imshow('frame', frame)
              
        # the 'q' button is set as the
        # quitting button you may use any
        # desired button of your choice
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        #time.sleep(0.01)
          
    # After the loop release the cap object
    vid.release()
    result.release()
    # Destroy all the windows
    cv2.destroyAllWindows()
    aruco.plot()
    
    return None


if __name__ == "__main__":

     pose = main(show = True)
    