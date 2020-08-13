# 2020/8/12 Update
This was a final project of an awesome course ENPM808X (Advanced Topics in Engineering; Software Development for Robotics) in 2017. I learned so much from this course and have been using some of the content during my everyday research programming tasks. This is a deprecated repository, but I just put it here to show my learning footprint.   

# Robotic-polishing
[![Build Status](https://travis-ci.org/michael081906/Robotic-polishing.svg?branch=master)](https://travis-ci.org/michael081906/Robotic-polishing)

## Overview and purpose of the project 

During a casting process such as solidifying metal plates, those metal plates usually have a coarse surface and is necessary to apply a polishing procedure to smooth the workpiece’s surface. Therefore, this project goal is to develop an industrial robotic application for polishing surface of a workpiece. The project consists of a robotic arm and a depth camera. The robotic arm will operate motion for polishing based on multiple positions sent from the depth camera. The depth camera will generate point cloud data of the object first and sends it to the robotic arm. The robotic arm will then start to polish the target surface by using the point cloud information. The module will be developed by using robot operating system(ROS) framework and simulating in gazebo.

## Presentation link:

Youtube:https://www.youtube.com/watch?v=YhQdyyhUI3c&feature=youtu.be
Google slides:https://docs.google.com/presentation/d/1rI3Nj8a8sGg6t7eYuaSVSCyE0983Q329ykSKCLgTq6U/edit#slide=id.p

## SIP process link:

SIP process: https://docs.google.com/spreadsheets/d/1UUcCnmibCKxxiiPX6WSljX4oefd2pRVzwdVSvT2h65M/edit#gid=0
Sprint reflection: https://docs.google.com/document/d/1ROcQN64o7sSzH2lZel_-hgHf1hhe9IFEqnSOjLyZbQA/edit

## Installation and build of package

In your catkin workspace directory (or create a new one)
```
git clone --recursive https://github.com/michael081906/Robotic-polishing.git
git clone --recursive https://github.com/michael081906/iiwa_stack.git
git clone --recursive https://github.com/ros-simulation/gazebo_ros_pkgs.git
git clone --recursive https://github.com/ros-controls/realtime_tools.git
git clone --recursive https://github.com/michael081906/control_toolbox.git
git clone --recursive https://github.com/ros-controls/ros_control.git
```
At catkin workspace 
```
cd ~/catkin_ws
catkin_make 
```

## Dependencies (and how to install if not included in repository)

Make sure you have these packages installed in the environment:
ros-kinetic-velocity-controllers
ros-kinetic-ros-control
ros-kinetic-position-controllers
ros-kinetic-joint-state-controller
ros-kinetic-joint-trajectory-controller

If not, type:

```
sudo apt-get install ros-kinetic-velocity-controllers ros-kinetic-ros-control ros-kinetic-position-controllers ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller
```
to install the packages.

## Run demo

After the Build step, switch to  catkin workspace and type:
```
roslaunch robotic_polishing robotPolishing.launch
```
to launch the gazebo environment. The open a new terminal and type:
```
rosrun robotic_polishing kukapcl.launch
```

First cloud viewer window shows the point cloud of the wheel taken by the depth camera. After closing the window (make sure click only ONE time on X button), the program will continue rest of the algorithm. Switch back to gazebo and you can now see the robot is moving.

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
 
## Doxygen documentation
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


## Dependencies

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


