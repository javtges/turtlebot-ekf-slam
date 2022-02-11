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


static double x_0, y_0, theta_0, x_length, y_length, motor_cmd_to_radsec, encoder_ticks_to_rad;
static double frequency;
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

    // convert to wheel_cmd
    wheel_speeds = drive.inverse_kinematics(twist);

    speeds.left_velocity = wheel_speeds.Ldot/motor_cmd_to_radsec;
    speeds.right_velocity = wheel_speeds.Rdot/motor_cmd_to_radsec;

    if(speeds.left_velocity > 256){
        speeds.left_velocity = 256;
    }
    if(speeds.left_velocity < -256){
        speeds.left_velocity = -256;
    }
    if(speeds.right_velocity > 256){
        speeds.right_velocity = 256;
    }
    if(speeds.right_velocity < -256){
        speeds.right_velocity = -256;
    }

}

void sensor_data_callback(const nuturtlebot_msgs::SensorData::ConstPtr& msg){
    //Joint states aren't being published when there's no sensor data, so we get problems in the tf tree because it's incomplete
    joint_states.header.stamp = ros::Time::now();
    joint_states.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
    joint_states.position = {toRadians(msg->left_encoder), toRadians(msg->right_encoder)};

    joint_states.velocity = {wheel_speeds.Ldot, wheel_speeds.Rdot};
    joint_states.effort = {0.0, 0.0};

}


TEST_CASE("testing turtle_interface subscribers and publishers", "[turtle_interface]"){

    // PUT NODEHANDLERS AND SUBSCRIBERS/PUBLISHERS HERE
    // ros::init(argc, argv, "turtle_interface");
    // ros::NodeHandle nh("~");
    ros::NodeHandle n;

    frequency = 0.25;
    x_0 = 0.0;
    y_0 = 0.0;
    theta_0 = 0.0;

    motor_cmd_to_radsec = 0.024;
    encoder_ticks_to_rad = 0.00153398;
    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    
    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    ros::Publisher wheel_speed_pub = n.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd",100);
    // ros::Subscriber wheel_speed_sub = n.subscribe("wheel_cmd",100,wheel_cmd_callback);

    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("red/joint_states",100);
    // ros::Publisher joint_state_sub = n.subscribe("red/joint_states",100,joint_state_callback);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel",100);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",100, cmd_vel_callback);

    ros::Publisher sensor_data_pub = n.advertise<nuturtlebot_msgs::SensorData>("sensor_data",100);
    ros::Subscriber sensor_data_sub = n.subscribe("sensor_data",100, sensor_data_callback);

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

        cmd_vel_pub.publish(input);
        
        r.sleep();
        ros::spinOnce();

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

        cmd_vel_pub.publish(input);
        
        r.sleep();
        ros::spinOnce();

        CHECK( speeds.left_velocity == Approx(-20));
        CHECK( speeds.right_velocity == Approx(20));
    }

    SECTION( "Testing sensor to joint_state conversion" ) {
        
        nuturtlebot_msgs::SensorData input;
        input.left_encoder = 2048;
        input.right_encoder = 2048;

        sensor_data_pub.publish(input);
        
        r.sleep();
        ros::spinOnce();

        CHECK( joint_states.position[0] == Approx(3.14159).margin(0.01));
        CHECK( joint_states.position[1] == Approx(3.14159).margin(0.01));
    }

}