# ME495 Sensing, Navigation and Machine Learning For Robotics
* Kevin Nella
* Winter 2022
# Brief Overview
The purpose of this project was to develop a custom C++ library to implement SLAM with a turtlebot 3 (ROS 2). The project was executed in sequential assignments throughout the ME 495 SLAM class. By the end of the class the project was capable of calculating the state of the robot and its environment (cylindrical obstacles) using odometry, laser scanning, and an Extended Kalman Filter.
# Package List
This repository consists of several ROS packages
- `nuturtle_description` - Edited turtlebot 3 burger urdf.xacro to launch one or multiple turtlebot nodes.
- turtlelib - C++ library to define, calculate, and output frame transformations and twists.
- nusim - Defines and launches nusim node, a simulated environment for the nuturtle.
- nuturtle_control - Control interface and odometry nodes for turtlebot simulation or real robot
- nuslam - Applied an Extended Kalman Filter with unknown datta association to correct odometry readings from turtlebot simulation or real robot.

Odometry Error:
dx = 9.09 CM
dy = 1.8 CM
dtheta = 41 degrees

EKF Filter Error:
Odometry to true:
dx = -0.044
dy = .032

SLAM Corrected to true:
dx = -0.0002
dy = -0.0003


<video src=https://user-images.githubusercontent.com/58793794/217950496-93bcb4ad-9c3e-4133-9091-a5ba8538c6b9.mp4>

<img src=https://user-images.githubusercontent.com/58793794/224528158-d2b79b79-33a8-4eda-8389-fc2c0f91fb32.png>

![Screenshot from 2023-03-11 23-12-10](https://user-images.githubusercontent.com/58793794/224997641-be58f4f3-b058-43a2-967a-ff4af58ee6c7.jpg)
[Screencast from 03-19-2023 04:29:16 PM.webm](https://user-images.githubusercontent.com/58793794/226211996-dfe8ae07-cbb5-46e5-9ec7-b2b9325d1f82.webm)
