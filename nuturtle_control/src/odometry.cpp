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
#include <turtlesim/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <nuturtlebot_msgs/SensorData.h>
#include <nuturtlebot_msgs/WheelCommands.h>

static std::string odom_frame, body_id, wheel_left, wheel_right;
static double x_0, y_0, theta_0, x_length, y_length, motor_cmd_to_radsec, encoder_ticks_to_rad;
static int frequency;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg){
    
}


int main(int argc, char * argv[])
{
    /// Initalize the node and the nodehandler, one is public and one is private.
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    n.param("frequency",frequency, 500);
    n.param("x0", x_0, 0.0);
    n.param("y0", y_0, 0.0);
    n.param("theta0", theta_0, 0.0);
    n.param("x_length", x_length, 10.0);
    n.param("y_length", y_length, 10.0);
    n.param("odom_id", odom_frame, std::string("odom"));
    
    if (!n.getParam("body_id",body_id)){
        ROS_ERROR_STREAM("Parameter not found!");
        ros::shutdown();
    }

    if (!n.getParam("wheel_left",wheel_left)){
        ROS_ERROR_STREAM("Parameter not found!");
        ros::shutdown();
    }

    if (!n.getParam("wheel_right",wheel_right)){
        ROS_ERROR_STREAM("Parameter not found!");
        ros::shutdown();
    }

    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    ros::Subscriber joint_state_sub = n.subscribe("/joint_states",100, joint_state_callback);


    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {
        // ts.data++;
        ros::spinOnce();
        r.sleep();
    }

    return 0;   
}