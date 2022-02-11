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
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <nuturtlebot_msgs/SensorData.h>
#include <nuturtlebot_msgs/WheelCommands.h>

/// \file
/// \brief The testfile for the odometry node.
///
/// PUBLISHES:
///     /cmd_vel (geometry_msgs::Twist): The commanded twist of the turtlebot.
///     /sensor_data (nuturtlebot_msgs::SensorData): The sensor data of the turtlebot. Here, only the wheel encoder ticks are used.
/// SUBSCRIBES:
///     /wheel_cmd (nuturtlebot_msgs::WheelCommands): The commanded wheel velocities in dynamixel ticks of the turtlebot.
///     /red/joint_states (sensor_msgs::JointStates): The joint states of the turtlebot.

static double x_0, y_0, theta_0, motor_cmd_to_radsec, encoder_ticks_to_rad;
static double frequency;
static turtlelib::DiffDrive drive;
static turtlelib::Q turtle_config;
static turtlelib::Phidot wheel_speeds;
static nuturtlebot_msgs::WheelCommands speeds;
static sensor_msgs::JointState joint_states;
static std::vector<double> positions, velocities;

/// \brief The callback function for the wheel_cmd subscriber
/// Reads the WheelCommand data required for the test cases
void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands & msg){
    
    speeds.left_velocity = msg.left_velocity;
    speeds.right_velocity = msg.right_velocity;
}

/// \brief The callback function for the joint_state subscriber
/// Reads the JointState data required for the test cases
void joint_state_callback(const sensor_msgs::JointState & msg){

    positions.resize(2);
    velocities.resize(2);
    positions = msg.position;
    velocities = msg.velocity;
}

TEST_CASE("testing turtle_interface subscribers and publishers", "[turtle_interface]"){

    ros::NodeHandle n;

    frequency = 100;
    x_0 = 0.0;
    y_0 = 0.0;
    theta_0 = 0.0;

    motor_cmd_to_radsec = 0.024;
    encoder_ticks_to_rad = 0.00153398;
    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    
    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    ros::Subscriber wheel_speed_sub = n.subscribe("wheel_cmd",100,wheel_cmd_callback);
    ros::Subscriber joint_state_sub = n.subscribe("red/joint_states",100,joint_state_callback);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Publisher sensor_data_pub = n.advertise<nuturtlebot_msgs::SensorData>("sensor_data",100);

    joint_states.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
    joint_states.position = {0.0, 0.0};
    joint_states.velocity = {0.0, 0.0};
    joint_states.effort = {0.0, 0.0};
    speeds.left_velocity = 0;
    speeds.right_velocity = 0;

    SECTION( "Testing cmd_vel to wheel_cmd, pure translation" ) {
        
        geometry_msgs::Twist input;

        input.linear.x = 0.2;
        input.linear.y = 0;
        input.linear.z = 0.0;
        input.angular.x = 0.0;
        input.angular.y = 0.0;
        input.angular.z = 0.0;

        for (int i=0; i<100; i++){
            cmd_vel_pub.publish(input);
            r.sleep();
            ros::spinOnce();

        }

        CHECK( speeds.left_velocity == Approx(252));
        CHECK( speeds.right_velocity == Approx(252));
    }

    SECTION( "Testing cmd_vel to wheel_cmd, pure rotation" ) {
        
        geometry_msgs::Twist input;

        input.linear.x = 0;
        input.linear.y = 0;
        input.linear.z = 0.0;
        input.angular.x = 0.0;
        input.angular.y = 0.0;
        input.angular.z = 0.2;

        for (int i=0; i<100; i++){
            cmd_vel_pub.publish(input);
            r.sleep();
            ros::spinOnce();

        }

        CHECK( speeds.left_velocity == Approx(-20));
        CHECK( speeds.right_velocity == Approx(20));
    }

    SECTION( "Testing sensor to joint_state conversion" ) {
        
        nuturtlebot_msgs::SensorData input;
        input.left_encoder = 2048;
        input.right_encoder = 2048;

        for (int i=0; i<100; i++){
            sensor_data_pub.publish(input);
            r.sleep();
            ros::spinOnce();

        }

        CHECK( positions[0] == Approx(3.14159).margin(0.01));
        CHECK( positions[1] == Approx(3.14159).margin(0.01));
    }

}