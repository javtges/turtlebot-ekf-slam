#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <string>
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <turtlesim/Pose.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <nuturtlebot_msgs/SensorData.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include "nuturtle_control/SetPose.h"
#include "nuslam/nuslam.hpp"
#include <armadillo>

static int frequency;
static int init_flag = 1;

void laser_scan_callback(const sensor_msgs::LaserScan & msg){
    ROS_WARN_STREAM("AAA");
}


int main(int argc, char * argv[]){

    ros::init(argc, argv, "landmarks");
    ros::NodeHandle n;

    frequency = 500;
    ros::Rate r(frequency);

    ros::Subscriber laser_scan_sub = n.subscribe("/laser_data", 10, laser_scan_callback);

    while(ros::ok()){

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
