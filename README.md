# Dynamic Path Planner
[![Build Status](https://travis-ci.org/urastogi885/optimal-path-finder.svg?branch=master)](https://travis-ci.org/github/urastogi885/dynamic-path-planner)

## Overview

This project used a dynamic programming (DP) approach to provide a collision-free optimal path to the robot. It 
implements the DP algorithm on differential-drive robot, namely, Turtlebot-3.

<p align="center">
  <img src="https://github.com/urastogi885/dynamic-path-planner/blob/master/images/gazebo_dynamic.gif">
  <br><b>Figure 1 - Turtlebot-3 navigating in a dynamic environment using dynamic programming algorithm</b><br>
</p>

The project was developed in various phases:

- The dynamic programming algorithm was tested in 2D static and dynamic environments.
- 3D environments were developed using ROS and Gazebo.
- The algorithm was tested in the 3D world with stationary as well as mobile obstacles.

The various GIF of each phase of testing can be found [here](https://github.com/urastogi885/dynamic-path-planner/tree/master/images).

## Team Members

- [Umang Rastogi](https://github.com/urastogi885)
- [Aman Virmani](https://github.com/AmanVirmani) 
- [Sayani Roy](https://github.com/sroy0108)

## Dependencies

- Ubuntu 18.04
- ROS Melodic
- Gazebo
- Turtlebot3 Packages
- Python Packages: Numpy, OpenCV-Python

## Install Dependencies

- Install Python3, Python3-tk, and the necessary libraries: (if not already installed)

```
sudo apt install python3 python3-tk
sudo apt install python3-pip
pip3 install numpy opencv-python
```

- Check if your system successfully installed all the dependencies
- Open terminal using Ctrl+Alt+T and enter python3.
- The terminal should now present a new area represented by >>> to enter python commands
- Now use the following commands to check libraries: (Exit python window using Ctrl+Z if an error pops up while running 
the below commands)

```
import tkinter
import numpy
import cv2
```

- Install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) by following the instructions given on referenced web-page.
- We recommend installing the full-desktop version of ROS because it automatically installs the latest compatible version of
Gazebo on your system.
- If you wish to install Gazebo separately, then follow the instruction on the [Gazebo install page](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
- Install Turtlebot-3 package and its dependencies using this [link](https://programmer.help/blogs/ubuntu-18.04-lts-melodic-ros-configuration-turtlebot-3-running-gazebo-simulation.html).

## Run Instructions

- Clone this repository in your ROS workspace
```
cd ~/<ROS_Workspace>/src/
git clone https://github.com/urastogi885/dynamic-path-planner
```
- You can also extract the compressed project file in the ```src``` folder of your ROS workspace.
- Open the terminal, build the ROS package, and run the launch file:
```
cd ~/<ROS_Workspace>
catkin_make or catkin build
source devel/setup.bash
roslaunch dynamic-path-planner planner.launch
```
- This would launch a gazebo world where the turtlebot-3 would reach its destination using the path planned via the 
dynamic programming algorithm.
