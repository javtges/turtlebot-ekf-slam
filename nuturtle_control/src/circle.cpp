#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <nuturtlebot_msgs/SensorData.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include "nuturtle_control/CircleControl.h"


/// \file
/// \brief Handles the odometry to allow the blue turtlebot to move in simulation and real life.
///
/// PARAMETERS:
///     ~frequency (int): Publishing frequency for cmd_vel
/// PUBLISHES:
///     /cmd_vel (geometry_msgs::Twist): The twist required to make the robot move in the given circle. Publishes 100 times per second.
/// SERVICES:
///     /circle_node/control (nuturtle_control::CircleControl): Calculates the twist required to move the turtlebot in a circle, and begins publishing if it wasn't already.
///     /circle_node/stop (std_srvs::Empty): Publishes 0 to all fields of cmd_vel, and stops publishing.
///     /circle_node/reverse (std_srvs::Empty): Reverses the direction of the turtlebot's movement.


static int frequency;
static geometry_msgs::Twist twist;
static bool stop_flag = false, go_flag = true;

/// \brief The callback for the control service. Sets the cmd_vel messages to be the twist requried to move the robot in a circle.
/// \param &Request - the inputs to the service. For this type there is a float64 radius and velocity inputs.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully
bool controlCallback(nuturtle_control::CircleControl::Request &Request, nuturtle_control::CircleControl::Response &){
    twist.angular.z = Request.velocity;
    twist.linear.x = Request.radius * Request.velocity;
    go_flag = true;
    stop_flag = false;
    return true;
}

/// \brief The callback for the stop service. Publishes 0 to cmd_vel and stops publishing.
/// \param &Request - the inputs to the service. For this type there are none.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully
bool stopCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &){
    stop_flag = true;
    twist.linear.x = 0;
    twist.linear.y = 0;
    twist.angular.z = 0;
    return true;
}

/// \brief The callback for the stop service. Reverses the values in cmd_vel.
/// \param &Request - the inputs to the service. For this type there are none.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully
bool reverseCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &){
    go_flag = true;
    stop_flag = false;
    twist.linear.x *= -1;
    twist.angular.z *= -1;
    return true;
}

/// The main function and loop.
int main(int argc, char * argv[])
{
    /// Initalize the node and the nodehandler, one is public and one is private.
    ros::init(argc, argv, "circle");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    nh.param("frequency",frequency, 100);

    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::ServiceServer reverseService = nh.advertiseService("reverse", reverseCallback);
    ros::ServiceServer stopService = nh.advertiseService("stop", stopCallback);
    ros::ServiceServer controlService = nh.advertiseService("control", controlCallback);

    twist.linear.x = 0.0;
    twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;


    /// The main loop of the node. Per the rate, this runs at 100Hz.
    while(ros::ok())
    {

        if(go_flag){
            cmd_vel_pub.publish(twist);
            if(stop_flag){
                go_flag = false;
            }
        }
        
        ros::spinOnce();
        r.sleep();
    }

    return 0;   
}