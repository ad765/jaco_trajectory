## Description

## Installation
Make sure to install the Jaco arm drivers and files, which are located in the kinova-ros repository. To install all the files, follow these steps (assuming your workspace is setup following the standard conventions):
```
cd ~/catkin_ws/src
git clone https://github.com/Kinovarobotics/kinova-ros.git
git clone https://github.com/ad765/jaco_trajectory.git
cd ~/catkin_ws
catkin_make
```

## Running the files
To run any ROS files, source all files first.
```
source /opt/ros/indigo/setup.bash
source devel/setup.bash
```
Start a roscore, and initialize the Gazebo model
```
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300
```
Open a new terminal and run the controller script
```
rosrun jaco_trajectory jaco_control.py
```

## Understanding the code
