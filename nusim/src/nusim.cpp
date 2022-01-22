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
#include <turtlesim/Pose.h>
#include "nusim/teleport.h"
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

/// \file
/// \brief Displays a red turtlebot in rviz, with obstacles in its vicinity.
///
/// PARAMETERS:
///     ~frequency (parameter_type): description of the parameter
///     ~radii (std::vector<double>): A vector of the radii for each of the cylindrical objects.
///     ~x_pos (std::vector<double>): A vector of the x positions for each of the objects.
///     ~y_pos (std::vector<double>): A vector of the y positions for each of the objects.
///     ~x0 (double): The starting x coordinate of the turtlebot.
///     ~y0 (double): The starting y coordinate of the turtlebot.
///     ~theta0 (double): The starting orientation (yaw) of the turtlebot.
/// PUBLISHES:
///     /nusim/obstacles (visualization_msgs::MarkerArray): Publishes and displays cylindrical markers in rviz that represents obstacles for the turtlebot. This is a latched publisher that publishes once with the markers being permenant.
///     /red/joint_states (sensor_msgs::JointState): Publishes joint states to the left and right wheels for the red turtlebot. Currently, joint states of 0 are published. This publishes 500 times per second.
///     /nusim/timestep (std_msgs::UInt64): Increments a 64-bit int that acts as the timestamp for the simulation. This timestamp updates 500 times per second.
/// SERVICES:
///     reset (std_srvs::Empty): teleports the turtlebot back to its starting position. additionally, resets the timestamp counter.
///     teleport (nusim::teleport): teleports the turtlebot to a given x, y, and theta pose.


static std_msgs::UInt64 ts;
static sensor_msgs::JointState jointState;
static geometry_msgs::TransformStamped transformStamped;
static visualization_msgs::MarkerArray ma;
static double x, y, theta;
static double x_0, y_0, theta_0;
std::vector<double> radii;
std::vector<double> x_locs;
std::vector<double> y_locs;


/// \brief The callback function for the reset service. Resets the timestamp counter and teleports the turtlebot back to its starting pose.
/// \param &Request - the inputs to the service. For this type there are none.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully
bool resetCallback(std_srvs::Empty::Request &Request, std_srvs::Empty::Response &Response){
    ts.data = 0;
    x = x_0;
    y = y_0;
    theta = theta_0;
    return true;
}


/// \brief The callback function for the teleport service. Teleports the turtlebot to a given pose.
/// \param &Request - the inputs to the service. For this type there is an x, y, and theta float64 input.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully
bool teleportCallback(nusim::teleport::Request &Request, nusim::teleport::Response &Response){
    x = Request.x;
    y = Request.y;
    theta = Request.theta;
    return true;
}

/// \brief Populates a MarkerArray message with the obstacles in the parameter server as specified by the yaml.
/// \param radii - a vector of the radii for each of the obstacles
/// \param x_locs - a vector of the x coordinates for each of the obstacles
/// \param y_locs - a vector of the y coordinates for each of the obstacles
/// returns the resulting MarkerArray ROS message.
visualization_msgs::MarkerArray addObstacles(std::vector<double> radii, std::vector<double> x_locs, std::vector<double> y_locs){

    int l = radii.size();
    visualization_msgs::MarkerArray maTemp;
    maTemp.markers.resize(l);

    for (int i=0; i<l; i++){
        
        maTemp.markers[i].header.frame_id = "world";
        maTemp.markers[i].header.stamp = ros::Time::now();
        maTemp.markers[i].ns = "obstacles";
        maTemp.markers[i].id = i;
        maTemp.markers[i].type = visualization_msgs::Marker::CYLINDER;
        maTemp.markers[i].action = visualization_msgs::Marker::ADD;

        maTemp.markers[i].pose.position.x = x_locs[i];
        maTemp.markers[i].pose.position.z = 0.125;
        maTemp.markers[i].pose.position.y = y_locs[i];
        maTemp.markers[i].pose.orientation.x = 0.0;
        maTemp.markers[i].pose.orientation.y = 0.0;
        maTemp.markers[i].pose.orientation.z = 0.0;
        maTemp.markers[i].pose.orientation.w = 1.0;

        maTemp.markers[i].scale.x = radii[i]*2;
        maTemp.markers[i].scale.y = radii[i]*2;
        maTemp.markers[i].scale.z = 0.25;

        maTemp.markers[i].color.r = 1.0;
        maTemp.markers[i].color.g = 0.0;
        maTemp.markers[i].color.b = 0.0;
        maTemp.markers[i].color.a = 1.0;
        maTemp.markers[i].lifetime = ros::Duration(0);
    }
    return maTemp;

}

/// The main function and loop.
int main(int argc, char * argv[])
{
    /// Initalize the node and the nodehandler, one is public and one is private.
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    int frequency;
    nh.param("frequency",frequency, 500);
    nh.param("x0", x_0, 0.0);
    nh.param("y0", y_0, 0.0);
    nh.param("theta0", theta_0, 0.0);

    nh.getParam("radii", radii);
    nh.getParam("x_pos", x_locs);
    nh.getParam("y_pos", y_locs);

    /// Setting up the looping rate and the required subscribers.
    ros::Rate r(frequency);
    ros::Publisher ts_pub = nh.advertise<std_msgs::UInt64>("timestep", frequency);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/red/joint_states",10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles",10, true);

    /// Setting up the services, and the robot's initial location.
    ros::ServiceServer resetService = nh.advertiseService("reset", resetCallback);
    ros::ServiceServer advertiseService = nh.advertiseService("teleport", teleportCallback);

    x = x_0; y = y_0; theta = theta_0;
    ts.data = 0;

    /// Populating the MarkerArray message and publishing it to display the markers.
    ma = addObstacles(radii, x_locs, y_locs);
    marker_pub.publish(ma);

    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {
        /// Creates a JointState message for the wheels.
        jointState.header.stamp = ros::Time::now();
        jointState.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
        jointState.position = {0.0, 0.0};
        jointState.velocity = {0.0, 0.0};
        jointState.effort = {0.0, 0.0};

        /// Set up a tf2 broadcaster to define the postion of the turtlebot.
        static tf2_ros::TransformBroadcaster br;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red-base_footprint";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        /// Send the transform and publish the JointState and the timestamp message.
        br.sendTransform(transformStamped);
        joint_state_pub.publish(jointState);
        ts_pub.publish(ts);

        /// Increment the timestamp, spin, and sleep for the 500Hz delay.
        ts.data++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}