# Introduction
Purpose of the project is to practice basic concepts of ROS. Therefore, scripts are generally written both in python and cpp and few of them are shown below as demonstration.
# Requirements 
<p>Ubuntu 20.04</p>
<p>ROS-Noetic</p>
<p>OpenCV</p>

# Installation
After satisfying requirements, first of all creating catkin folder is required which will contain this project. Steps are given below to run projects succesfully.

**Create a container folder** `mkdir -p myros/src`

**Open .bashrc** `nano ~/.bashrc`

**Add folder path to bashrc** `source /home/enis/myros/devel/setup.bash`

**Change directory into myros and initalize catkin**`cd myros && catkin_make`

**Copy ros_projects folder into src/** `cp -r /home/enis/ROS_Basic_Projects/ros_projects src/`

**Build files** `catkin_make`

# Running an Python Script
**Terminal 1** `roscore`

**Terminal 2** `rosrun ros_projects tennis_ball_publisher.py`

**Terminal 3** `rosrun ros_projects tennis_ball_subscriber.py`

### Running an Cpp Script
First of all be sure about the cpp file which will be used is defined in `CMakeLists.txt` in ros_projects folder and built with `catkin_make` command.
**Terminal 1** `roscore`

**Terminal 2** `rosrun ros_projects image_pub`

**Terminal 3** `rosrun ros_projects image_sub`

# Demonstrations
### Service-Client
<p align="center">
   <img src="https://user-images.githubusercontent.com/45767042/131259189-c0c7f698-4dfd-44b0-9bcd-9ea406f73909.png", width=1280, height=720>
</p>

### Turtlesim Controlling

<p align="center">
  <img src="https://user-images.githubusercontent.com/45767042/131259095-ef3fc2f1-5181-4c9f-8b1d-c8384ca508a3.png", width=1280, height=720>
</p>

### Ball Tracking
<p align="center">
   <img src="https://user-images.githubusercontent.com/45767042/131257683-ee23ce9a-cf72-4574-b53d-c649805cc898.png", width=1280, height=720>
</p>

# Reference
Main referencce of my projects is https://www.udemy.com/course/ros-essentials/
