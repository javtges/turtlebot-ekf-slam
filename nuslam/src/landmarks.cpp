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
// static std::vector<std::vector<float>> clusters;

void laser_scan_callback(const sensor_msgs::LaserScan & msg){
    ROS_WARN_STREAM("AAA");

    int length = msg.ranges.size();
    double threshold = 0.2;
    float range = 0.0, prev_range = 0.0, prev_distance = 0.0, distance = 0.0;
    std::vector<std::vector<turtlelib::Vector2D>> clusters;
    int n_clusters = 0;

    for(int i=0; i<length; i++){
        
        // Check the value at the angle and the value behind it
        // If they're similar, add to cluster
        // If not, make a new cluster

        range = msg.ranges[i];
        
        if(i==0){
            prev_range = msg.ranges[length-1];
            double mx = std::cos(turtlelib::deg2rad(i)) * (prev_range);
            double my = std::sin(turtlelib::deg2rad(i)) * (prev_range);

            prev_distance = std::sqrt( std::pow(mx,2) + std::pow(my,2));
        }
        else{
            prev_range = msg.ranges[i-1];
            double mx = std::cos(turtlelib::deg2rad(i)) * (prev_range);
            double my = std::sin(turtlelib::deg2rad(i)) * (prev_range);

            prev_distance = std::sqrt( std::pow(mx,2) + std::pow(my,2));
        }

        double mx = std::cos(turtlelib::deg2rad(i)) * (range);
        double my = std::sin(turtlelib::deg2rad(i)) * (range);

        distance = std::sqrt( std::pow(mx,2) + std::pow(my,2));

        double dist_diff = std::abs(distance - prev_distance);

        
        if (range > 0.0){
            ROS_WARN_STREAM(i << " " << range << " " << mx << " " << my << " distance " << distance);
            
            if(n_clusters == 0){  // First cluster
                    std::vector<turtlelib::Vector2D> new_cluster;
                    new_cluster.push_back ({mx, my});
                    clusters.push_back (new_cluster);
                    n_clusters++;
            }
            else{ // Already existing clusters
                if (dist_diff < threshold){ // If similar to previous
                    // Add to cluster
                    clusters.at(n_clusters-1).push_back ({mx, my});
                }
                else{ // Make a new cluster
                    std::vector<turtlelib::Vector2D> new_cluster;
                    new_cluster.push_back ({mx, my});
                    clusters.push_back (new_cluster);
                    ROS_ERROR_STREAM("new cluster!");
                    n_clusters++;
                }
            }
        }

        // Compare first and last. If close, merge. Otherwise, don't
        
        
    }    
    turtlelib::Vector2D first = clusters.at(0).at(0);

    std::vector<turtlelib::Vector2D> first_cluster = clusters.at(0);
    std::vector<turtlelib::Vector2D> last_cluster = clusters.at(n_clusters-1);

    ROS_WARN_STREAM( first_cluster.size() << " " << last_cluster.size());

    double first_distance = std::sqrt( std::pow(first.x,2) + std::pow(first.y,2) );
    if (std::abs(first_distance - distance) < threshold){
        clusters.at(0).insert( clusters.at(0).end(), clusters.at(n_clusters-1).begin(), clusters.at(n_clusters-1).end() );
        clusters.pop_back(); // Removes last cluster
        ROS_WARN_STREAM("overlap clusters, " << clusters.at(0).size());
        
    }
    
    ROS_WARN_STREAM(clusters.size());


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
