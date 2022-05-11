#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 22 23:41:06 2022

@author: user
"""

# import the necessary packages
import cv2

def main():
    

    continue_capture = True
    num = 0
    
    # define a video capture object
    vid = cv2.VideoCapture(0)
    
    vid.set(cv2.CAP_PROP_FRAME_WIDTH, 1600)
    vid.set(cv2.CAP_PROP_FRAME_HEIGHT, 1200)

    while continue_capture:
        
        # Capture frame
        ret, frame = vid.read()


        # display the image on screen and wait for a keypress
        cv2.imshow("Image", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
        if cv2.waitKey(1) & 0xFF == ord('w'):
            # Save image
            print(cv2.imwrite(f'image_{num}.jpeg',frame))
            print("Saved!")
            num += 1


if __name__ == "__main__":
    main()
