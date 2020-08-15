## 2020/8/12 Update
This was a final project of an awesome course ENPM808X (Advanced Topics in Engineering; Software Development for Robotics) in 2017. I learned so much from this course and have been using some of the contents during my everyday research programming tasks. This is a deprecated repository, but I just put it here to show my learning footprint.   

###TODO:
1. iiwa_stack has error with "iiwa_ros/command/joint_position.hpp" during build. 


# Robotic-polishing
[![Build Status](https://travis-ci.org/michael081906/ros-project-robotic-polishing.svg?branch=master)](https://travis-ci.org/michael081906/ros-project-robotic-polishing)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

<img src="https://github.com/michael081906/ros-project-robotic-polishing/blob/master/docs/robot-polishing-demo.gif" width="400" >  

## Overview

During a casting process such as solidifying metal plates,the metal plates usually have a coarse surface and it's necessary to apply a polishing procedure to smooth the workpiece’s surface. Therefore, the goal of this project is to develop a robotic application for polishing surface. The project consists of a robotic arm and a depth camera. The robotic arm will operate motions for polishing based on multiple positions sent from the depth camera. The depth camera will generate pointcloud data of the object first and sends it to the robotic arm. The robotic arm will then start to polish the target surface by using the pointcloud information. The module was developed by using Robot Operating System(ROS) with Kinetic version and simulated in gazebo. 

## Presentation
- [Google slides](https://docs.google.com/presentation/d/1rI3Nj8a8sGg6t7eYuaSVSCyE0983Q329ykSKCLgTq6U/edit#slide=id.p)

## SIP Process

- [SIP process](https://docs.google.com/spreadsheets/d/1UUcCnmibCKxxiiPX6WSljX4oefd2pRVzwdVSvT2h65M/edit#gid=0)
- [Sprint reflection](https://docs.google.com/document/d/1ROcQN64o7sSzH2lZel_-hgHf1hhe9IFEqnSOjLyZbQA/edit)

## Installation

In your catkin workspace directory (or create a new one)
```
git clone --recursive https://github.com/michael081906/ros-project-robotic-polishing.git
```
Make sure you have the dependencies install
```
cd ~/catkin_ws/src
bash ./ros-project-robotic-polishing/dependencies_install.bash
```
Go back to the catkin workspace 
```
cd ~/catkin_ws
catkin_make 
```

## Run Demo

After the Build step, switch to catkin workspace and type:
```
roslaunch robotic_polishing robotPolishing.launch
```
to launch the gazebo environment. The open a new terminal and type:
```
rosrun robotic_polishing kukapcl.launch
```

First cloud viewer window shows the pointcloud of the wheel taken by the depth camera. After closing the window (make sure you click only ONCE on X button), the program will continue. Switch back to gazebo and you can now see the robot is moving.

If you would like to record by using rosbag, you can type:
```
roslaunch robotic_polishing kukapcl.launch rosbag:=1
```
to record all the topic without /camera and /camera_ir

## RosTest

At catkin workspace 
```
cd ~/catkin_ws
catkin_make run_tests
```
If the test fail with no reason, try to re-run the test after several seconds.  
 
## Doxygen Documentation
```
sudo apt install doxygen
cd <path to repository>
mkdir docs
cd docs
doxygen -g Robotic-polishing
```
open the Robotic-polishing file(which is a configuration file of doxygen), and modify input tag into

INPUT                  = ../include ../src ../test

and then save the file. Go back to the terminal and type:
```
doxygen Robotic-polishing
```
find an index.html in ./html directory, which can be viewed on web browser.


## License 

- Point Cloud Library(pcl) License: Copyright (c) 2012-, Open Perception, Inc.
- Doxygen license: Copyright © 1997-2016 by Dimitri van Heesch.
- Googletest license: Copyright 2008, Google Inc.
- Delaunay triangulation S-hull license: Copyright 2016 Dr David Sinclair
- iiwa_stack license: Copyright (c) 2016-2017, Salvatore Virga - salvo.virga@tum.de


## Dependencies List

- Kinetic version of Robot operating system(ROS) framework under Ubuntu 16.04 LTS.  
- Rviz and Gazebo
- Eclipse
- CMake
- Google Test Framework for unit testing. Copyright (c) 2008, Google Inc.
- Point cloud library(PCL) for point cloud processing. BSD License. Copyright (c) 2012-2017, Open Perception, Inc.
- ROS stack: https://github.com/SalvoVirga/iiwa_stack.git BSD License. Copyright (c) 2016-2017, Salvatore Virga.
- Github: https://github.com/michael081906/MidtermProject.git BSD License. Copyright (c) 2017, Michael Kam.
- Doxygen: Copyright © 1997-2016 by Dimitri van Heesch.
- Delunay triangulation S-hull license: Copyright 2016 Dr David Sinclair


