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
#include "nuturtle_control/SetPose.h"

/// \file
/// \brief Handles the odometry to allow the blue turtlebot to move in simulation and real life.
///
/// PARAMETERS:
///     /nusim/frequency (parameter_type): description of the parameter
///     /nusim/frequency/x0 (double): The starting x coordinate of the turtlebot.
///     /nusim/frequency/y0 (double): The starting y coordinate of the turtlebot.
///     /nusim/frequency/theta0 (double): The starting orientation (yaw) of the turtlebot.
///     /odom_id (std::string): The name of the odometry frame, coincident with the world frame.
///     /body_ (std::string): The name of the body frame for the odometry.
///     /wheel_left (std::string): The name of the left wheel joint.
///     /wheel_right (std::string): The name of the right wheel joint.
/// PUBLISHES:
///     /odom (nav_msgs::Odometry): The odometry of the blue turtlebot. Publishes 500 times per second.
/// SUBSCRIBES:
///     /red/joint_states (sensor_msgs::JointStates): The joint states of the turtlebot.
/// SERVICES:
///     set_pose (nuturtle_control::SetPose): Changes the odometry so that the robot thinks it's at the given pose.


static std::string odom_frame, body_id, wheel_left, wheel_right;
static double x_0, y_0, theta_0;
static int frequency;
static turtlelib::DiffDrive drive;
static turtlelib::Q turtle_config;
static turtlelib::Phidot wheel_speeds;
static nuturtlebot_msgs::WheelCommands speeds;
static sensor_msgs::JointState joint_states;
static nav_msgs::Odometry odom;
static std::vector<double> positions, velocities;


/// \brief The callback function for the joint_state subscriber
/// Calculates the new turtlebot configuration, determines the instanteous twist, and begins populating the odometry message.
void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg){
    // Update internal odometry state

    turtlelib::Phidot currentSpeeds;
    turtlelib::Phi currentAngles, nextAngles;
    turtlelib::Twist2D twist;
    turtlelib::Q new_config;

    positions.resize(2);
    velocities.resize(2);
    positions = msg->position;
    velocities = msg->velocity;

    currentAngles.L = positions[0];
    currentAngles.R = positions[1];
    currentSpeeds.Ldot = velocities[0];
    currentSpeeds.Rdot = velocities[1];

    drive.setSpeeds(currentSpeeds);
    drive.setAngles(currentAngles);

    nextAngles.L = (currentSpeeds.Ldot/frequency) + currentAngles.L;
    nextAngles.R = (currentSpeeds.Rdot/frequency) + currentAngles.R;

    twist = drive.get_twist_from_angles(currentSpeeds);

    drive.forward_kinematics(nextAngles);

    ROS_ERROR_STREAM("ODOM Twist" << twist << "\r\n");

    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.linear.y = twist.ydot;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = twist.thetadot;

}


/// \brief The callback for the set_pose service
/// Sets the new configuration of the turtlebot to the service request data
/// \param &Request - the inputs to the service. For this type there is an x, y, and theta float64 input.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully
bool set_poseCallback(nuturtle_control::SetPose::Request &Request, nuturtle_control::SetPose::Response &){
    // Reset the position of the odometry according to the request.
    turtlelib::Q newPose;
    newPose.x = Request.x;
    newPose.y = Request.y;
    newPose.theta = Request.theta;
    drive.setConfig(newPose);

    return true;
}

/// The main function and loop
int main(int argc, char * argv[])
{
    /// Initalize the node and the nodehandler, one is public and one is private.
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    n.param("/nusim/frequency",frequency, 500);
    n.param("/nusim/x0", x_0, 0.0);
    n.param("/nusim/y0", y_0, 0.0);
    n.param("/nusim/theta0", theta_0, 0.0);
    n.param("odom_id", odom_frame, std::string("odom"));

    turtlelib::Q initial_config;
    initial_config.theta = theta_0;
    initial_config.x = x_0;
    initial_config.y = y_0;
    
    if (!n.getParam("body_id",body_id)){
        ROS_ERROR_STREAM("Body ID frame not found!");
        ros::shutdown();
    }

    if (!n.getParam("wheel_left",wheel_left)){
        ROS_ERROR_STREAM("Wheel Left frame not found!");
        ros::shutdown();
    }

    if (!n.getParam("wheel_right",wheel_right)){
        ROS_ERROR_STREAM("Wheel Right frame not found!");
        ros::shutdown();
    }

    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    ros::Subscriber joint_state_sub = n.subscribe("red/joint_states",10, joint_state_callback);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",100);
    ros::ServiceServer setPoseService = nh.advertiseService("set_pose", set_poseCallback);

    drive.setConfig(initial_config);

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {

        /// Get the current config of the turtlebot
        turtle_config = drive.getConfig();

        /// Make the transform from the odom frame to the body frame
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = body_id;
        transformStamped.transform.translation.x = turtle_config.x;
        transformStamped.transform.translation.y = turtle_config.y;
        transformStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, turtle_config.theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        br.sendTransform(transformStamped);


        /// Make and publish the odometry message using the current blue turtle configuration
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "world"; //previously odom_frame
        odom.child_frame_id = body_id;
        odom.pose.pose.position.x = turtle_config.x;
        odom.pose.pose.position.y = turtle_config.y;
        odom.pose.pose.position.z = 0;
        tf2::Quaternion quat;
        quat.setRPY(0,0,turtle_config.theta);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();

        // ROS_WARN_STREAM("publishing odom??");
        odom_pub.publish(odom);

        ros::spinOnce();
        r.sleep();
    }

    return 0;   
}