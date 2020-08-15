#!/bin/bash
git clone --recursive https://github.com/IFL-CAMP/iiwa_stack.git
sudo apt install ros-kinetic-control-toolbox ros-kinetic-ros-control ros-kinetic-gazebo-ros-pkgs ros-kinetic-realtime-tools 
sudo apt-get install ros-kinetic-velocity-controllers ros-kinetic-ros-control ros-kinetic-position-controllers ros-kinetic-joint-state-controller ros-kinetic-joint-trajectory-controller
#git clone --branch kinetic-devel --recursive https://github.com/ros-simulation/gazebo_ros_pkgs.git
#git clone --branch kinetic-devel --recursive https://github.com/ros-controls/realtime_tools.git
#git clone --branch --recursive https://github.com/michael081906/control_toolbox.git
#git clone --branch --recursive https://github.com/ros-controls/ros_control.git
