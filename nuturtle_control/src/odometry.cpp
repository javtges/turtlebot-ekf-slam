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

static std::string odom_frame, body_id, wheel_left, wheel_right;
static double x_0, y_0, theta_0, motor_cmd_to_radsec, encoder_ticks_to_rad;
static int frequency;
static turtlelib::DiffDrive drive;
static turtlelib::Q turtle_config;
static turtlelib::Phidot wheel_speeds;
static nuturtlebot_msgs::WheelCommands speeds;
static sensor_msgs::JointState joint_states;
static nav_msgs::Odometry odom;
static std::vector<double> positions, velocities;

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
    // ROS_ERROR_STREAM(currentAngles.L);
    currentAngles.R = positions[1];
    currentSpeeds.Ldot = velocities[0];
    currentSpeeds.Rdot = velocities[1];

    drive.setSpeeds(currentSpeeds);
    drive.setAngles(currentAngles);

    // ROS_ERROR_STREAM("!!!!!!!!!!!!!!!!!!");
    // MAYBE NEED TO HANDLE ANGLE ROLLOVER HERE????

    nextAngles.L = (currentSpeeds.Ldot/frequency) + currentAngles.L;
    nextAngles.R = (currentSpeeds.Rdot/frequency) + currentAngles.R;

    twist = drive.get_twist_from_angles(currentSpeeds);
    // UNCOMMENT THIS LATER
    // Tried: nextAngles, twist, current/lastangles
    new_config = drive.forward_kinematics(nextAngles);

    // ROS_ERROR_STREAM(new_config.x);

    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = body_id;
    odom.pose.pose.position.x = new_config.x;
    odom.pose.pose.position.y = new_config.y;
    odom.pose.pose.position.z = 0;
    tf2::Quaternion quat;
    quat.setRPY(0,0,new_config.theta);
    odom.pose.pose.orientation.x = quat.x();
    odom.pose.pose.orientation.y = quat.y();
    odom.pose.pose.orientation.z = quat.z();
    odom.pose.pose.orientation.w = quat.w();

    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.linear.y = twist.ydot;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = twist.thetadot;

    // use delta of wheel positions (with no 2pi rollover??) to find the new configuration
    // use the twist velocity from the wheelspeeds to set the odom twist

}

bool set_poseCallback(nuturtle_control::SetPose::Request &Request, nuturtle_control::SetPose::Response &Response){
    // Reset the position of the odometry according to the request.
    turtlelib::Q newPose;
    newPose.x = Request.x;
    newPose.y = Request.y;
    newPose.theta = Request.theta;
    drive.setConfig(newPose);

    return true;
}


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

        turtle_config = drive.getConfig();

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = odom_frame;
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

        // ROS_WARN_STREAM("publishing odom??");
        odom_pub.publish(odom);

        ros::spinOnce();
        r.sleep();
    }

    return 0;   
}