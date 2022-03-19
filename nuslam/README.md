# Nuslam
An implimentation of Extended Kalman Filter (EKF) SLAM, and machine learning for data association on the turtlebot3.

## To launch the SLAM:
* `roslaunch nuslam unknown_data_assoc.launch` to launch the SLAM pipeline in Rviz. To use the real turtlebot, run `roslaunch nuslam unknown_data_assoc.launch cmd_src:=teleop robot:=turtlebot` where "turtlebot" is the name of the robot to be launched on, after cross-compilation.

* To view the landmarks as detected by the LiDAR sensor, run `roslaunch nuslam landmark_detection.launch` to see the markers in yellow.

## To edit parameters of the simulator:
* `config/basic_world.yaml` contains information specifying the robot's starting location, and the obstacles' x coordinate, y coordinate, and radius. An arbitrary amount of obstacles can be added.

A final video of the EKF SLAM in operation with unknown data association in RViz can be shown here:

https://youtu.be/o4k6hs_qkiI

Here is the SLAM without data association - using omniscient knowledge of where the markers are in the world.

https://youtu.be/XMKJ3N6Otlw

This can be run on the real turtlebot as well, however there were some issues with the turtlebots themselves and the functionality for the LiDAR.