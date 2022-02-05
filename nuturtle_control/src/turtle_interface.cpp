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

static double x_0, y_0, theta_0, x_length, y_length, motor_cmd_to_radsec, encoder_ticks_to_rad;
static int frequency;
static turtlelib::DiffDrive drive;
static turtlelib::Q turtle_config;
static turtlelib::Phidot wheel_speeds;
static nuturtlebot_msgs::WheelCommands speeds;
static sensor_msgs::JointState joint_states;


int toEncoderTicks(double radians){
    return (int)(radians/encoder_ticks_to_rad) % 4096;
}

double toRadians(int ticks){
    return ticks * encoder_ticks_to_rad;
}

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    turtlelib::Twist2D twist;
    twist.xdot = msg->linear.x;
    twist.ydot = msg->linear.y;
    twist.thetadot = msg->angular.z;
    wheel_speeds = drive.inverse_kinematics(twist);
    speeds.left_velocity = wheel_speeds.Ldot;
    speeds.right_velocity = wheel_speeds.Rdot;
}

void sensor_data_callback(const nuturtlebot_msgs::SensorData::ConstPtr& msg){
    joint_states.header.stamp = ros::Time::now();
    joint_states.name = {"wheel_left_joint", "wheel_right_joint"};
    joint_states.position = {toRadians(msg->left_encoder), toRadians(msg->right_encoder)};

    // May need to use another method for calculating velocity
    joint_states.velocity = {speeds.left_velocity, speeds.right_velocity};
    joint_states.effort = {0.0, 0.0};

}


int main(int argc, char * argv[])
{
    /// Initalize the node and the nodehandler, one is public and one is private.
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;


    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    n.param("frequency",frequency, 500);
    n.param("x0", x_0, 0.0);
    n.param("y0", y_0, 0.0);
    n.param("theta0", theta_0, 0.0);
    n.param("x_length", x_length, 10.0);
    n.param("y_length", y_length, 10.0);
    
    if (!n.getParam("motor_cmd_to_radsec",motor_cmd_to_radsec)){
        ROS_ERROR_STREAM("Parameter motor_cmd_to_radsec not found!");
        ros::shutdown();
    }

    if (!n.getParam("encoder_ticks_to_rad",encoder_ticks_to_rad)){
        ROS_ERROR_STREAM("Parameter encoder_ticks_to_rad not found!");
        ros::shutdown();
    }

    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    ros::Publisher wheel_speed_pub = n.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd",100);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states",100);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",100, cmd_vel_callback);
    ros::Subscriber sensor_data_sub = n.subscribe("sensor_data",100, sensor_data_callback);


    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {
        wheel_speed_pub.publish(speeds);
        joint_state_pub.publish(joint_states);
        ros::spinOnce();
        r.sleep();
    }

    return 0;   
}