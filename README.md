# ECE-CUP-2021
Autonomous robot capable of finding and grabing cube from camera feed

The objective is to retrieve cubes in a particular order and bring them back to a precise area, with an autonomous robot. 
The robot is driven by an ESP68266 card communicating via wifi with a computer on which the code is executed. The computer is connected to an IP camera that films the board in which the robot evolves.

## Features
 - Detection of the position and orientation of the slave robot using an arUco code
 - Detection of the position and sorting of the cubes using openCV according to their colors
 - Creation of a trajectory vector between the robot and the next target
 - PID in position and rotation of the robot from the camera to follow the trajectory
 - Tracking of the opponent robot from a position selected at the beginning of the program, and predict his trajectory by derivating the pos
 - Dodge the opponent robot using its position and the A* algorithm. 
 ![image](https://user-images.githubusercontent.com/93131053/219491715-4ada151a-629e-4e3d-86f9-2c1a3f57dce4.png)

 - creation of a strategy to recover the cubes to maximize the number of points depending on the distance, combos ...
 
 - UDP connection between robot and computer for command communication

 - code on ESP68266 for network communication, motor driver and last centimeter approach with ultrasound sensor. It receive commands from the computer and execute them. 
 
 ## Setup
 
 - Download open CV and build it 
1 - https://www.youtube.com/watch?v=x5EWlNQ6z5w
2  - https://www.youtube.com/watch?v=p-6rG6Zgu4U ( for Visual studio only ) 

build project from source code
for testing purpose, You can use OBS ( https://obsproject.com/fr/download ) virtual camera as a camera and create a scene with asset from the repo
