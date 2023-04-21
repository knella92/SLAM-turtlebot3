# ME495 Sensing, Navigation and Machine Learning For Robotics
* Kevin Nella
* Winter 2022
# Brief Overview
The purpose of this project was to develop a custom C++ library to implement SLAM with a turtlebot 3 (ROS 2). The project was executed in sequential assignments throughout the ME 495 SLAM class. By the end of the class the project was capable of calculating the state of the robot and its environment (cylindrical obstacles) using odometry, laser scanning, and an Extended Kalman Filter.
# Package List
This repository consists of several ROS packages
- `nuturtle_description` - Edited turtlebot 3 burger urdf.xacro to launch one or multiple turtlebot nodes, visualized in RViz.
- `turtlelib` - C++ library to define, calculate, and output frame transformations and twists. Computes inverse kinematics from odometry as well as forwrad kinematics. Also contains functions for Extended Kalman Filter and a circle detection algorithm.
- `nusim` - Defines and launches nusim node, a simulated environment for the nuturtle.
- `nuturtle_control` - Control interface and odometry nodes for turtlebot simulation or real robot
- `nuslam` - Applied an Extended Kalman Filter with unknown data association to correct odometry readings from turtlebot simulation or real robot.

Odometry Error:
dx = 9.09 CM
dy = 1.8 CM
dtheta = 41 degrees

EKF Filter Error:
Odometry to true:
dx = -0.044 CM
dy = .032 CM

SLAM Corrected to true:
dx = -0.0002 CM
dy = -0.0003 CM

<video src=https://user-images.githubusercontent.com/58793794/217950496-93bcb4ad-9c3e-4133-9091-a5ba8538c6b9.mp4>
<br>
![Screenshot from 2023-03-11 23-12-10](https://user-images.githubusercontent.com/58793794/233505100-d10a511e-e4f5-4aca-b24e-8d5cb2ec0f6e.jpg)
