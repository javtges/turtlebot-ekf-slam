#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>





int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;

    // read parameters, create publishers/subscribers
    int frequency = 500; //nh.getParam("~frequency");

    ros::Rate r(frequency);

    ros::Publisher ts_pub = nh.advertise<std_msgs::UInt64>("ts", frequency);

    std_msgs::UInt64 ts;
    ts.data = 0;


    while(ros::ok())
    {

        ts_pub.publish(ts);

        ts.data++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}