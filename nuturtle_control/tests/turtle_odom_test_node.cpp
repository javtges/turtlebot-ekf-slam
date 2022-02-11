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

/// \file
/// \brief The testfile for the odometry node.
///
/// SUBSCRIBES:
///     /odom (nav_msgs::Odometry): The odometry of the turtlebot.


static std::string odom_frame, body_id;
static double x_0, y_0, theta_0, motor_cmd_to_radsec, encoder_ticks_to_rad, frequency;
static turtlelib::DiffDrive drive;
static turtlelib::Q turtle_config;
static turtlelib::Phidot wheel_speeds;
static nuturtlebot_msgs::WheelCommands speeds;
static sensor_msgs::JointState joint_states;
static nav_msgs::Odometry odom;
static std::vector<double> positions, velocities;


/// \brief The callback function for the odometry subscriber
/// Reads the odometry data required for the test cases
void odom_callback(const nav_msgs::Odometry & msg){
    turtle_config.x = msg.pose.pose.position.x;
    turtle_config.y = msg.pose.pose.position.y;
    turtle_config.theta = 0;
}

TEST_CASE("testing turtle_interface subscribers and publishers", "[turtle_interface]"){

    // PUT NODEHANDLERS AND SUBSCRIBERS/PUBLISHERS HERE
    ros::NodeHandle n;

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

        /// Initializes the transform listener to find the data required for the test case.
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