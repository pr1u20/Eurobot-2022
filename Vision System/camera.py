#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 22 23:41:06 2022

@author: user
"""

# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2


class Camera_obj():

    "Capture image from the camera"

    def __init__(self):

        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.rawCapture = PiRGBArray(self.camera)

    def capture_frame(self):

        # grab an image from the camera
        self.camera.capture(self.rawCapture, format="bgr")
        image = self.rawCapture.array

        self.rawCapture.truncate(0)

        return image

def save_continue(image, num):
    "Check if user wants to save and/or continue taking pictures"

    user_input_1 = input("Save the image (y/n): ")

    if user_input_1 == "y":
        # Save image
        print(cv2.imwrite(f'image_{num}.jpeg',image))
        print("Saved!")

    else:
        print("Image not saved.")

    user_input_2 = input("Continue taking pictures (y/n): ")

    if user_input_2 == "y":
        print("More photos!")
        continue_capture = True

    else:
        continue_capture = False

    return continue_capture



def main(fps):
    
    # fps = frames per second

    cam = Camera_obj()
    continue_capture = True
    num = 0
    

    while continue_capture:
        
        # Capture frame
        image = cam.capture_frame()
        
        # allow the camera to warmup
        time.sleep(1 / fps)

        # display the image on screen and wait for a keypress
        cv2.imshow("Image", image)
        cv2.waitKey(0)

        continue_capture = save_continue(image, num)

        num += 1



if __name__ == "__main__":
    main(10)