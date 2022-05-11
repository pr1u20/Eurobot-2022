#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar  6 10:03:05 2022

@author: user
"""


from aruco_rasp import Aruco, Camera_obj, mtx, dst, aruco_marker_side_length
import Sream_reduced as stream

import cv2

    
        
def main(show = False):

    aruco = Aruco(mtx, dst, aruco_dict_type = cv2.aruco.DICT_4X4_100, length_marker = aruco_marker_side_length)
    
    cam = Camera_obj()

    while(True):
        
        # Read frame
        frame = cam.capture_frame()
        
        # allow the camera to warmup
#         time.sleep(0.1)
        
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
        
    # Destroy all the windows
    cv2.destroyAllWindows()

    return None


if __name__ == "__main__":

     main(show = True)
    
