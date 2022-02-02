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
// #include <nuturtlebot_msgs/SensorData.h>
// #include <nuturtlebot_msgs/WheelCommands.h>

static double x_0, y_0, theta_0, x_length, y_length, motor_cmd_to_radsec, encoder_ticks_to_rad;
static int frequency;

void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& msg){

}

// void sensor_data_callback(const nuturtlebot_msgs::SensorData::ConstPtr& msg){
// 
// }


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
        ROS_ERROR_STREAM("Parameter not found!");
        ros::shutdown();
    }

    if (!n.getParam("encoder_ticks_to_rad",encoder_ticks_to_rad)){
        ROS_ERROR_STREAM("Parameter not found!");
        ros::shutdown();
    }



    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency); 
    
    // ros::Publisher sensor_data_pub = n.advertise<nuturtlebot::WheelCommands>("/wheel_cmd",100);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states",100);
    ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel",100, cmd_vel_callback);
    // ros::Subscriber sensor_data_sub = n.subscribe("/sensor_data",100, sensor_data_callback);


    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {
        // ts.data++;
        ros::spinOnce();
        r.sleep();
    }

    return 0;   
}