## Description
Trajectory controller for Jaco 6DOF arm. Includes inverse kinematics for optimized trajectory.

## Installation
The following tutorial assumes that all files and packages are created using the "catkin" method. Make sure to install the Jaco arm drivers and files, which are located in the kinova-ros repository. To install all the files, follow these steps (assuming your workspace is setup following the standard conventions):
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
Start a roscore and initialize the Gazebo model
```
roslaunch kinova_gazebo robot_launch.launch kinova_robotType:=j2n6s300
```
Open a new terminal and run the controller script
```
rosrun jaco_trajectory jaco_control.py
```

## Using the code
The unedited code offers capabilities to move from an initial state to a final state in a linear trajectory.

### Parameters
The parameters chosen are to ensure the stability of the solution. The inverse kinematics algorithm uses a pseudo-inverse jacobian to calculate state transitions.

- numPts: the number of points that characterizes the path from the initial state to the final state. A larger value gives more accuracy along the path, but is more computationally expensive. This is different from the variable "step" in the sense that it provides global accuracy
- step: the increment for state transition. A larger value will allow the state to change rapidly, but may induce instabilities. For more accuracy, use a small value, but this will increase computational expensiveness.
- error: the threshold error between the current (x,y,z) position and the desired (x,y,z) position of the end affector. Measured in meters, default error is 1 mm.

### Initial state
Initial state is given by the qi0 variables. All these values are given in radians. With the configuration given, the inverse kinematics algorithm should hold work without any numerical instabilities. In the case that the system becomes unstable, use the transpose calculation in the inverseKinematics function from robot_functions.py.

### Final state
The final state is defined by its position in (x,y,z) space. Change these variables to redefine the final end affector coordinates.

### Discretized path
The discretized path can be changed to a different path. The default is a line, with goal points defined by a linear spacing from "numPts". Currently not parameterized.
