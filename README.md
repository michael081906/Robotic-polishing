# Robotic-polishing
[![Build Status](https://travis-ci.org/michael081906/Robotic-polishing.svg?branch=master)](https://travis-ci.org/michael081906/Robotic-polishing)

## Overview and purpose of the project 

During a casting process such as solidifying metal plates, those metal plates usually have a coarse surface and is necessary to apply a polishing procedure to smooth the workpiece’s surface. Therefore, this project goal is to develop an industrial robotic application for polishing surface of a workpiece. The project consists of a robotic arm and a depth camera. The robotic arm will operate motion for polishing based on multiple positions sent from the depth camera. The depth camera will generate point cloud data of the object first and sends it to the robotic arm. The robotic arm will then start to polish the target surface by using the point cloud information. The module will be developed by using robot operating system(ROS) framework and simulating in gazebo.


## SIP process link:

https://docs.google.com/spreadsheets/d/1UUcCnmibCKxxiiPX6WSljX4oefd2pRVzwdVSvT2h65M/edit#gid=0

## Dependencies

◆ Kinetic version of Robot operating system(ROS) framework under Ubuntu 16.04 LTS.
◆ Rviz and Gazebo
◆ Eclipse
◆ CMake
◆ Google Test Framework for unit testing. Copyright (c) 2008, Google Inc.
◆ Point cloud library(PCL) for point cloud processing. BSD License. Copyright (c) 2012-2017, Open Perception, Inc.
◆ ROS stack: https://github.com/SalvoVirga/iiwa_stack.git BSD License. Copyright (c) 2016-2017, Salvatore Virga.
◆ Github: https://github.com/michael081906/MidtermProject.git BSD License. Copyright (c) 2017, Michael Kam.
◆ Doxygen: Copyright © 1997-2016 by Dimitri van Heesch.


