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


/// \file
/// \brief Handles publishers and subscribers to allow the turtlebot to move in simulation and real life.
///
/// PARAMETERS:
///     /frequency (parameter_type): description of the parameter
///     /x0 (double): The starting x coordinate of the turtlebot.
///     /y0 (double): The starting y coordinate of the turtlebot.
///     /theta0 (double): The starting orientation (yaw) of the turtlebot.
///     /motor_cmd_to_radsec (double): The motor command to radians/second conversion factor.
///     /encoder_ticks_to_rad (double): The encoder ticks to radians conversion factor.
/// PUBLISHES:
///     /wheel_cmd (nuturtlebot_msgs::WheelCommands): The wheel velocities in dynamixel ticks, this is read by the turtlebot to make the wheels move. Publishes 500 times per second.
///     /red/joint_states (sensor_msgs::JointStates): The instanteous wheel positions and velocities of the turtlebot. This updates 500 times per second.
/// SUBSCRIBES:
///     /cmd_vel (geometry_msgs::Twist): The body twist of the turtlebot.
///     /sensor_data (nuturtlebot_msgs::SensorData): The positions of the turtlebot wheels in encoder ticks, along with other unused sensor data from the turtlebot.


static double x_0, y_0, theta_0, motor_cmd_to_radsec, encoder_ticks_to_rad;
static int frequency;
static turtlelib::DiffDrive drive;
static turtlelib::Q turtle_config;
static turtlelib::Phidot wheel_speeds;
static nuturtlebot_msgs::WheelCommands speeds;
static sensor_msgs::JointState joint_states;

/// \brief Converts a radians measurement to turtlebot3 wheel encoder ticks.
/// \param radians - the angle in radians
/// returns the corresponding amount of encoder ticks
int toEncoderTicks(double radians){
    return (int)(radians/encoder_ticks_to_rad) % 4096;
}

/// \brief Converts an encoder ticks measurement to radians.
/// \param ticks - the angle in encoder ticks
/// returns the corresponding amount of radians
double toRadians(int ticks){
    return ticks * encoder_ticks_to_rad;
}

/// \brief The callback function for the wheel_command subscriber
/// Sets the turtlebot wheel speeds in dynamixel ticks that correspond to the twist given by the /cmd_vel topic
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    turtlelib::Twist2D twist;
    twist.xdot = msg->linear.x;
    twist.ydot = msg->linear.y;
    twist.thetadot = msg->angular.z;

    // convert to wheel_cmd
    wheel_speeds = drive.inverse_kinematics(twist);

    speeds.left_velocity = wheel_speeds.Ldot/motor_cmd_to_radsec;
    speeds.right_velocity = wheel_speeds.Rdot/motor_cmd_to_radsec;

    //publish, sleep, spinOnce
    // void callbacks, TEST_CASE with node handles, publishers, subscribers, SECTIONS with tests 
    // .test file does something weird in cmakelists, launchfile that includes start robot nusim and then the tests.cpp file

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


/// \brief The callback function for the sensor_data subscriber
/// Converts the encoder data to joint state positions and velocities
void sensor_data_callback(const nuturtlebot_msgs::SensorData::ConstPtr& msg){
    //Joint states aren't being published when there's no sensor data, so we get problems in the tf tree because it's incomplete
    joint_states.header.stamp = ros::Time::now();
    joint_states.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
    joint_states.position = {toRadians(msg->left_encoder), toRadians(msg->right_encoder)};
    // ROS_ERROR_STREAM("oh no");
    // May need to use another method for calculating velocity
    joint_states.velocity = {wheel_speeds.Ldot, wheel_speeds.Rdot};
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
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("red/joint_states",100);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",100, cmd_vel_callback);
    ros::Subscriber sensor_data_sub = n.subscribe("sensor_data",100, sensor_data_callback);

    joint_states.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
    joint_states.position = {0.0, 0.0};
    joint_states.velocity = {0.0, 0.0};
    joint_states.effort = {0.0, 0.0};
    speeds.left_velocity = 0;
    speeds.right_velocity = 0;


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