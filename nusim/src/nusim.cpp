#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include "nusim/teleport.h"

static std_msgs::UInt64 ts;
static sensor_msgs::JointState jointState;
// static tf2_ros::TransformBroadcaster br;
static geometry_msgs::TransformStamped transformStamped;
static double x, y, theta;


bool resetCallback(std_srvs::Empty::Request &Request, std_srvs::Empty::Response &Response){
    ts.data = 0;
    return true;
}

bool teleportCallback(nusim::teleport::Request &Request, nusim::teleport::Response &Response){
    x = Request.x;
    y = Request.y;
    theta = Request.theta;
    return true;
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    // read parameters, create publishers/subscribers
    int frequency = 500;
    nh.setParam("frequency", 500);

    int fq;
    nh.getParam("frequency", fq);

    ros::Rate r(frequency);

    ros::Publisher ts_pub = nh.advertise<std_msgs::UInt64>("timestep", fq);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("joint_states",1);
    ros::ServiceServer resetService = nh.advertiseService("reset", resetCallback);
    ros::ServiceServer advertiseService = nh.advertiseService("teleport", teleportCallback);

    x = 0; y = 0; theta = 0;
    ts.data = 0;


    while(ros::ok())
    {

        jointState.header.stamp = ros::Time::now();
        jointState.name = {"red_wheel_left_joint", "red_wheel_right_joint"};
        jointState.position = {0.0, 0.0};
        jointState.velocity = {0.0, 0.0};
        jointState.effort = {0.0, 0.0};

        static tf2_ros::TransformBroadcaster br;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red_base_footprint";
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