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
#include "nusim/teleport.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>


// static std_msgs::UInt64 ts;
// static sensor_msgs::JointState jointState;
// static geometry_msgs::TransformStamped transformStamped;
// static visualization_msgs::MarkerArray ma, walls;
// static double x, y, theta, x_length, y_length;
// static double x_0, y_0, theta_0;
// std::vector<double> radii;
// std::vector<double> x_locs;
// std::vector<double> y_locs;



int main(int argc, char * argv[])
{
    /// Initalize the node and the nodehandler, one is public and one is private.
    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    // /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    // n.param("/nusim/frequency",frequency, 500);
    // n.param("x0", x_0, 0.0);
    // n.param("y0", y_0, 0.0);
    // n.param("theta0", theta_0, 0.0);
    // n.param("x_length", x_length, 10.0);
    // n.param("y_length", y_length, 10.0);

    // n.getParam("radii", radii);
    // n.getParam("x_pos", x_locs);
    // n.getParam("y_pos", y_locs);

    // /// Setting up the looping rate and the required subscribers.
    // ros::Rate r(frequency); 
    // ros::Publisher ts_pub = nh.advertise<std_msgs::UInt64>("timestep", frequency);
    // ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/red/joint_states",10);
    // ros::Publisher obs_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles",10, true); //True means latched publisher
    // ros::Publisher wall_pub = nh.advertise<visualization_msgs::MarkerArray>("walls",10, true); //True means latched publisher
    // ros::Publisher wheel_cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel",100); 
    
    
    // // ros::Publisher sensor_data_pub = nh.advertise<nuturtlebot::WheelCommands>("/red/sensor_data",100);
    // // ros::Subscriber wheel_cmd_sub = n.subscribe("/red/wheel_cmd",100, wheelCallback);


    // /// Setting up the services, and the robot's initial location.
    // ros::ServiceServer resetService = nh.advertiseService("reset", resetCallback);
    // ros::ServiceServer advertiseService = nh.advertiseService("teleport", teleportCallback);

    // x = x_0; y = y_0; theta = theta_0;
    // ts.data = 0;

    // /// Populating the MarkerArray message and publishing it to display the markers.
    // ma = addObstacles(radii, x_locs, y_locs);
    // walls = addWalls(x_length, y_length);
    // obs_pub.publish(ma);
    // wall_pub.publish(walls);

    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {
        // ts.data++;
        ros::spinOnce();
        // r.sleep();
    }

    return 0;   
}