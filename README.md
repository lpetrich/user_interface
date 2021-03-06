# ROS QT User Interface Package for Assistive Robotics

## Introduction
 Substantial research has been published in Human Robot Interaction (HRI), tracking, and visual servoing over the past decade. We present an intuitive system based on the user-centered design for setting and executing visual tasks. Our HRI system expands on the work by Gridseth et al., incorporating visual task specification, registration based tracking and uncalibrated image- based visual servoing.

## Required Prerequisites:
* [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu)
* [MTF](https://github.com/abhineet123/MTF)
* [Qt](https://www.qt.io/download) 
	- version 4.8.6 used during development
* [OpenCV](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html) 
	- version 2.4.8 used during development
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page) 
	- version 3.2.10 used during development


## Required ROS Packages:
* [mtf_bridge](https://github.com/lpetrich/mtf_bridge)
* [geometric_error](https://github.com/lpetrich/geometric_error)

## Usage:
```bash
# Steps to create a new catkin workspace and install packages
source /opt/ros/indigo/setup.bash
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
cd src
git clone https://github.com/lpetrich/user_interface.git
git clone https://github.com/lpetrich/geometric_error.git
git clone https://github.com/lpetrich/mtf_bridge.git
cd ..
catkin_make
# To use in a preexisting workspace, cd to the src folder and follow the last 5 steps

# To run, use one of the following commands
roslaunch user_interface single.launch
roslaunch user_interface stereo.launch

```

## Additional Resources:
* [Website](https://sites.google.com/ualberta.ca/49918-lpetrich/home)

* [Report](https://github.com/lpetrich/user_interface/blob/master/ProjectReport499W18.pdf)
