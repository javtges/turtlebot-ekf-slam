#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>


static std_msgs::UInt64 ts;


bool resetCallback(std_srvs::Empty::Request &Request, std_srvs::Empty::Response &Response){
    ts.data = 0;
    return true;
}



int main(int argc, char * argv[])
{
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");

    // read parameters, create publishers/subscribers
    int frequency = 500;
    nh.setParam("frequency", 500);

    int fq;
    nh.getParam("frequency", fq);

    ros::Rate r(frequency);

    ros::Publisher ts_pub = nh.advertise<std_msgs::UInt64>("timestep", fq);
    ros::ServiceServer service = nh.advertiseService("reset", resetCallback);

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