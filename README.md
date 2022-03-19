# ME495 Sensing, Navigation and Machine Learning For Robotics
* James Avtges
* Winter 2022
# Package List
This repository consists of several ROS packages:
- nuturtle_description - Displays multiple turtlebot3 models in rviz, each appearing with a different color with physical properties of the robot customizable.
- nusim - Displays a red turtlebot in rviz, with 3 customizable cylindrical obstacles also displayed. The robot can teleport to a location and reset itself.
- nuturtle_control - Controls the turtlebot with differential drive kinematics. Can run on the real turtlebot, or simulated in rviz, and calculates & displays odometry.
- turtlesim - Contains the C++ libraries required for the nuturtle_control package.
- nuslam - Contains the C++ libraries and nodes required for Extended Kalman Filter (EKF) SLAM on turtlebot3. LiDAR data is used to classify circular objects the scene and then the measurement is associated with either an unknown object or a known one.

A final video of the EKF SLAM in operation with unknown data association in RViz can be shown here:

https://youtu.be/o4k6hs_qkiI

Here is the SLAM without data association - using omniscient knowledge of where the markers are in the world.

This can be run on the real turtlebot as well, however there were some issues with the turtlebots themselves and the functionality for the LiDAR.