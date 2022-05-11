# Eurobot 2022
## _Team Lèa_

[![Build Status](https://travis-ci.org/joemccann/dillinger.svg?branch=master)](https://travis-ci.org/joemccann/dillinger)
Team members: Pedro Rodriguez, Fraser Heatlie, Gopal Virk, Michael Collier, Dun Liu

Eurobot 2022 is a student robotics contest, where robots have to compete autonomously in the table of the image to get the maximum number of points. 

![map copy](https://user-images.githubusercontent.com/73555876/163551793-5a57cdea-362d-4e41-af64-7638e1adfb31.png)
## Background
Our team, Lèa, competed in the UK Finals, and finished 4th. We lost unfairly in the knockout round of the quarterfinals. It says clearly in the rules that "the reveal of a team’s red square invalidate all the team’s excavation squares", but the judges seemed to have forgotten the rules. Our team uses two different robots and a central tracking device.

This repository consists of three parts:
- Vision System
- Robots
- Visualization

## Vision System
This is our central tracking device, which consists of a camera connected to a Raspberry Pi and a nrf24lo1+ transciever with a view of the whole table. 

The video images are processed in real time and ArUco tags are detected. The position and orientation of the tags are estimated with respect to the camera frame of reference. The rotation vector (Rodrigues), quaternions and Euler transformations are used to change the axis of reference from the camera to the ArUco tag with ID 42 located in the centre of the Eurobot table. The table is treated as a plane and the z-axis is ignored. Each robot has a specific ArUco tag on top and its position and orientation is calculated with respect to ID 42. After detection, a single string sends the position and orientation of both robots with the NRF24lO1+ transceiver. Each robot has a NRF24lO1+ transceiver and knows which part of the string corresponds to its position. 

In order to run the code that processes the images and sends the position and orientation to the robots, run the following command inside the vision system folder in the Raspberry Pi:
```sh
sudo -E python3 aruco_RP_cv2.py
```
Run the following command to only process the images:
```sh
sudo python3 arcuco_pose_estimation.py
```
## Robots
This consists of two files, one for the arduino in Robot 1, whose movement map is seen on the following image:
![Screen Shot 2022-05-09 at 08 22 02](https://user-images.githubusercontent.com/73555876/167871986-824575e2-8b19-46d6-a238-e756cf64ddc5.png)

And another for Robot 2, whose movement map is seen in the following image.
![Screen Shot 2022-05-09 at 08 22 11](https://user-images.githubusercontent.com/73555876/167871829-8a81158e-c411-4051-8dee-1369a6ac289e.png)

At any given point during the match the robot calculates in real time the angle and distance that it needs to move to get to the next position. This facilitates the integration with the vision system. The codes includes a sense and avoid system, and can control a resistance reader system and three gripper. 
## Visualization
The last folder is just a jupyter notebook that allows us to visualize the datapoints through which the robot passes. This helps us to quickly make changes in the movement of the robot.
