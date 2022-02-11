#include <ros/ros.h>
#include <catch_ros/catch.hpp>
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
#include <tf2_ros/transform_listener.h>
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


static std::string odom_frame, body_id;
static double x_0, y_0, theta_0, motor_cmd_to_radsec, encoder_ticks_to_rad, frequency;
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
    currentAngles.R = positions[1];
    currentSpeeds.Ldot = velocities[0];
    currentSpeeds.Rdot = velocities[1];

    drive.setSpeeds(currentSpeeds);
    drive.setAngles(currentAngles);

    nextAngles.L = (currentSpeeds.Ldot/frequency) + currentAngles.L;
    nextAngles.R = (currentSpeeds.Rdot/frequency) + currentAngles.R;

    twist = drive.get_twist_from_angles(currentSpeeds);
    // Tried: nextAngles, twist, current/lastangles
    new_config = drive.forward_kinematics(nextAngles);

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

}

void odom_callback(const nav_msgs::Odometry & msg){
    turtle_config.x = msg.pose.pose.position.x;
    turtle_config.y = msg.pose.pose.position.y;
    turtle_config.theta = 0;
}

TEST_CASE("testing turtle_interface subscribers and publishers", "[turtle_interface]"){

    // PUT NODEHANDLERS AND SUBSCRIBERS/PUBLISHERS HERE
    // ros::init(argc, argv, "turtle_interface");
    // ros::NodeHandle nh("~");
    ros::NodeHandle n;

    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    odom_frame = "odom";
    body_id = "blue-base_footprint";

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    frequency = 0.25;
    x_0 = 0.0;
    y_0 = 0.0;
    theta_0 = 0.0;

    motor_cmd_to_radsec = 0.024;
    encoder_ticks_to_rad = 0.00153398;

    turtlelib::Q initial_config;
    initial_config.theta = theta_0;
    initial_config.x = x_0;
    initial_config.y = y_0;

    drive.setConfig(initial_config);

    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    ros::Publisher wheel_speed_pub = n.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd",100);

    ros::Subscriber joint_state_sub = n.subscribe("red/joint_states",10, joint_state_callback);
    ros::Subscriber odom_callback_sub = n.subscribe("/odom",10, odom_callback);

    ros::ServiceClient setPoseClient = n.serviceClient<nuturtle_control::SetPose>("/odometry_node/set_pose");

    SECTION( "Testing the set_pose service" ) {

        nuturtle_control::SetPose input;
        input.request.x = 2.0;
        input.request.y = 1.0;
        input.request.theta = 0.0;

        setPoseClient.waitForExistence();
        
        if(setPoseClient.call(input)){
        }
        else{
            ROS_ERROR_STREAM("IT'S DOOMED");
        }

        r.sleep();
        ros::spinOnce();
        
        CHECK(turtle_config.x == 2.0);
        CHECK(turtle_config.y == 1.0);
        
    }

    SECTION( "Testing the transform broadcaster from odom to blue-base_footprint"){

        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped transformStamped;

        r.sleep();
        ros::spinOnce();

        transformStamped = tfBuffer.lookupTransform("odom", "blue-base_footprint", ros::Time(0));

        CHECK(transformStamped.transform.translation.x == 2);
        CHECK(transformStamped.transform.translation.y == 1);

    }


}