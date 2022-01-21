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

static std_msgs::UInt64 ts;
static sensor_msgs::JointState jointState;
static geometry_msgs::TransformStamped transformStamped;
static visualization_msgs::MarkerArray ma;
static double x, y, theta;
static double x_0, y_0, theta_0;
std::vector<double> radii;
std::vector<double> x_locs;
std::vector<double> y_locs;

bool resetCallback(std_srvs::Empty::Request &Request, std_srvs::Empty::Response &Response){
    ts.data = 0;
    x = x_0;
    y = y_0;
    theta = theta_0;
    return true;
}

bool teleportCallback(nusim::teleport::Request &Request, nusim::teleport::Response &Response){
    x = Request.x;
    y = Request.y;
    theta = Request.theta;
    return true;
}


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


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    ros::NodeHandle ob("~obstacles");
    ros::NodeHandle n;

    // read parameters, create publishers/subscribers
    int frequency;
    nh.param("frequency",frequency, 500);
    nh.param("x0", x_0, 0.0);
    nh.param("y0", y_0, 0.0);
    nh.param("theta0", theta_0, 0.0);

    nh.getParam("radii", radii);
    nh.getParam("x_pos", x_locs);
    nh.getParam("y_pos", y_locs);

    ros::Rate r(frequency);

    ros::Publisher ts_pub = nh.advertise<std_msgs::UInt64>("timestep", frequency);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states",10);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles",10, true);

    ros::ServiceServer resetService = nh.advertiseService("reset", resetCallback);
    ros::ServiceServer advertiseService = nh.advertiseService("teleport", teleportCallback);

    x = x_0; y = y_0; theta = theta_0;
    ts.data = 0;

    ma = addObstacles(radii, x_locs, y_locs);
    marker_pub.publish(ma);


    while(ros::ok())
    {

        jointState.header.stamp = ros::Time::now();
        jointState.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
        jointState.position = {0.0, 0.0};
        jointState.velocity = {0.0, 0.0};
        jointState.effort = {0.0, 0.0};

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
     
        br.sendTransform(transformStamped);
        joint_state_pub.publish(jointState);
        ts_pub.publish(ts);

        ts.data++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}