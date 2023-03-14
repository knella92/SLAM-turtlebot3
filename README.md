# ME495 Sensing, Navigation and Machine Learning For Robotics
* Kevin Nella
* Winter 2022
# Package List
This repository consists of several ROS packages
- nuturtle_description - Edited turtlebot 3 burger urdf.xacro to launch one or multiple turtlebot nodes.
- turtlelib - C++ library to define, calculate, and output frame transformations and twists.
- nusim - Defines and launches nusim node, a simulated environment for the nuturtle.
- nuturtle_control - Control interface and odometry nodes for turtlebot simulation or real robot
- nuslam - Apply an Extended Kalman Filter to correct odometry readings from turtlebot simulation or real robot.

Odometry Error:
dx = 9.09 CM
dy = 1.8 CM
dtheta = 41 degrees


<video src=https://user-images.githubusercontent.com/58793794/217950496-93bcb4ad-9c3e-4133-9091-a5ba8538c6b9.mp4/>

![Screenshot from 2023-03-11 23-12-10](https://user-images.githubusercontent.com/58793794/224997641-be58f4f3-b058-43a2-967a-ff4af58ee6c7.jpg)
