#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  8 14:03:37 2022

@author: user
"""

from arcuco_pose_estimation import Aruco, mtx, dst, aruco_marker_side_length

import Sream_reduced as stream
import cv2


def main(show = False):

    aruco = Aruco(mtx, dst, aruco_dict_type = cv2.aruco.DICT_4X4_100, length_marker = aruco_marker_side_length)
  
    # define a video capture object
    vid = cv2.VideoCapture(0)
    
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)
    
    if not vid.isOpened():
        print("Cannot open camera")

    while(True):
        
        # Capture the video frame
        # by frame
        ret, frame = vid.read()
        
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break
        
        # allow the camera to warmup
        # time.sleep(0.1)
        
        # Process frame
        frame, AB_pose = aruco.live_processing(frame)

        # Create packet to send
        if AB_pose[2] < 0:  AB_pose[2] = 360 + AB_pose[2]
        if AB_pose[5] < 0:  AB_pose[5] = 360 + AB_pose[5]
            
        AX = "{:04n}".format(int(AB_pose[0]))
        AY = "{:04n}".format(abs(int(AB_pose[1])))
        AO = "{:03n}".format(int(AB_pose[2]))
        BX = "{:04n}".format(int(AB_pose[3]))
        BY = "{:04n}".format(abs(int(AB_pose[4])))
        BO = "{:03n}".format(int(AB_pose[5]))
        packet = f'LAX{AX}Y{AY}O{AO}BX{BX}Y{BY}O{BO}E'
        print(packet)
        while len(packet) < 32:
            packet += "0"
        buffer = packet.encode('UTF-8')
        #buffer = b"LAX1500Y1700O180BX1360Y1230O172E"

        # Send data to arduinos
        stream.master(buffer)

        # display the frame on screen and wait for a keypress
        if show:
            cv2.imshow('frame', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
    # After the loop release the cap object
    vid.release()
    # Destroy all the windows
    cv2.destroyAllWindows()

    return None


if __name__ == "__main__":

     main(show = True)
    

