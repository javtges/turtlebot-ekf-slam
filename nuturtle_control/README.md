# nuturtle_control package
A package for controlling turtlebots in simulation and real life. Calculates odometry, simulated odometry, and sends commands to make the real turtlebot move in a circle, under teleoperated control, or other methods.

# Components

## Nodes
- turtle_interface - Performs conversions to adapt the turtlebot to cmd_vel commands, and reads the turtlebot's sensor data.
- odometry - Calculates the odometry of the turtlebot
- Provides various services to move the turtlebot in a circle.

## Services
- /circle_node/control - Commands the robot to move in a circle of specified radius (in meters) and roatational velocity. Appropriate values for velocity are generally under 1 rad/second.
- /circle_node/reverse - Reverses the direction of the turtlebot if it's moving in a circle.
- /circle_node/stop - Stops the turtlebot. 
- /odometry_node/set_pose - Sets the odometry to be a user-specified pose.


## Usage

To launch, call the launchfile `start_robot.launch`. Arguments for the launchfile include:

* `cmd_src` - Controls the input of cmd_vel commands. 'circle' moves the robot in a circle, 'teleop' allows user control, and 'none' starts no cmd_vel publishers.

* `use_rviz` - Determines if rviz is to be launched. Default is true.

* `robot` - Determines the robot to run. The default is 'nusim' for simulation, 'localhost' to run the nodes from the turtlebot3, or a turtlebot name to run the nodes on a specific robot.

If `use_rviz` is `true`, rviz will launch and display a blue robot. This represents the odometry of the turtlebot, and will move when the real turtlebot moves. If `robot` is set to `nusim`, then a red turtlebot will also display in rviz, representing the simulated position of the turtlebot. Provided no `/nusim/teleport` or `/odometry_node/set_pose` calls occur, the red and blue turtlebots will be coincident with each other.

A sample call of the launchfile is:

`roslaunch nuturtle_control start_robot.launch cmd_src:=circle use_rviz:=true robot:=casey`


To perform unit tests on the differential drive kinematics, 2D transformation math, and turtle control publishers and subscribers, run `catkin_make run_tests` in the base directory of the workspace.



# Media and Comments

* [Moving forward and backwards (teleop) - Rviz](https://youtu.be/EZUhVQ0WLWw)
* [Moving forward and backwards (teleop) - Turtlebot](https://youtu.be/t2TfCA_ihj4)

The final odometry provides the following pose:

position: 

      x: 0.08044509589672089
      y: 0.0
      z: 0.0

orientation: 

      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0

There is some error from the real turtlebot position to the calculated odometry, though overall it is rather accurate.

* [Spinning in place (teleop) - Rviz](https://youtu.be/jD496T7PCvk)
* [Spinning in place (teleop) - Turtlebot](https://youtu.be/qb4rqiRPXIA)

The final odometry provides the following pose:

position: 

      x: -7.37094524083659e-05
      y: 0.0
      z: 0.0

orientation: 

      x: 0.0
      y: 0.0
      z: 0.3826665134750049
      w: 0.9238865403635254

There is some error from the real turtlebot position to the calculated odometry, there appears to be more error from wheel slippage when spinning in place. There is also some human error relating to the timing of when the turtlebot is stopped.

* [Moving in a circle (cmd_src) - Rviz](https://youtu.be/sp8PxFXIsgA)
* [Moving in a circle (teleop) - Turtlebot](https://youtu.be/CK-Iqv82AXU)

The final odometry provides the following pose:

 position: 

      x: -0.0056962245143949986
      y: 9.462991874897853e-05
      z: 0.0

orientation: 

      x: 0.0
      y: 0.0
      z: -0.01419995017087547
      w: 0.999899175624795

There is some human error relating to the timing of when the turtlebot is stopped, as the odometry is nearly zeroed while the turtlebot is a bit off its starting position. This is due to wheel slippage, however the odometry appears to be rather accurate.

* [Moving in a circle with poor odometry (cmd_src) - Rviz](https://youtu.be/CMNxz9pU7iQ)
* [Moving in a circle with poor odometry (teleop) - Turtlebot](https://youtu.be/If_OGGZAWGs)

The final odometry provides the following pose:

position: 

      x: -0.09790478646755219
      y: 0.1203593760728836
      z: 0.0

orientation: 

      x: 0.0
      y: 0.0
      z: -0.7757498855354423
      w: 0.6310405019424253

Here, the odometry is signifigantly worse because of the faster speed and tighter circle, which causes more wheel slippage.