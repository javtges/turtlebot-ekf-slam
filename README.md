# ME495 Sensing, Navigation and Machine Learning For Robotics
* James Avtges
* Winter 2022
# Package List
This repository consists of several ROS packages:
- nuturtle_description - Displays multiple turtlebot3 models in rviz, each appearing with a different color with physical properties of the robot customizable.
- nusim - Displays a red turtlebot in rviz, with 3 customizable cylindrical obstacles also displayed. The robot can teleport to a location and reset itself.
- nuturtle_control - Controls the turtlebot with differential drive kinematics. Can run on the real turtlebot, or simulated in rviz, and calculates & displays odometry.
- turtlesim - Contains the C++ libraries required for the nuturtle_control package.