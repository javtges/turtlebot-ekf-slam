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
static double max_range, min_range;

void laser_scan_callback(const sensor_msgs::LaserScan & msg){
    ROS_WARN_STREAM("AAA");

    int length = msg.ranges.size();
    double threshold = 0.2;
    float range = 0.0, prev_range = 0.0, prev_distance = 0.0, distance = 0.0;
    std::vector<std::vector<turtlelib::Vector2D>> clusters;
    int n_clusters = 0;

    for(int i=0; i<length; i++){ // loop through all angles
        
        // Check the value at the angle and the value behind it
        // If they're similar, add to cluster
        // If not, make a new cluster

        range = msg.ranges[i];
        
        if(i==0){
            prev_range = msg.ranges[length-1];
            // double mx = std::cos(turtlelib::deg2rad(i)) * (prev_range);
            // double my = std::sin(turtlelib::deg2rad(i)) * (prev_range);

            // prev_distance = std::sqrt( std::pow(mx,2) + std::pow(my,2));
        }
        else{
            prev_range = msg.ranges[i-1];
            // double mx = std::cos(turtlelib::deg2rad(i)) * (prev_range);
            // double my = std::sin(turtlelib::deg2rad(i)) * (prev_range);

            // prev_distance = std::sqrt( std::pow(mx,2) + std::pow(my,2));
        }

        double mx = std::cos(turtlelib::deg2rad(i)) * (range);
        double my = std::sin(turtlelib::deg2rad(i)) * (range);

        double dist_diff = std::abs(range - prev_range);
        // ROS_ERROR_STREAM(min_range << " " << max_range);
        if ((min_range < range) && (range < max_range)){
            ROS_WARN_STREAM(i << " " << range << " " << mx << " " << my);
            
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

    // std::vector<turtlelib::Vector2D> first_cluster = clusters.at(0);
    // std::vector<turtlelib::Vector2D> last_cluster = clusters.at(n_clusters-1);

    // ROS_WARN_STREAM( first_cluster.size() << " " << last_cluster.size());

    double first_distance = std::sqrt( std::pow(first.x,2) + std::pow(first.y,2) );
    if (std::abs(first_distance - distance) < threshold){
        clusters.at(0).insert( clusters.at(0).end(), clusters.at(n_clusters-1).begin(), clusters.at(n_clusters-1).end() );
        clusters.pop_back(); // Removes last cluster
        ROS_WARN_STREAM("overlap clusters, " << clusters.at(0).size());
    }
    ROS_WARN_STREAM(clusters.size());

    for (int j=0; j<clusters.size(); j++){ // loop through clusters
        int n = clusters.at(j).size();
        double x_bar = 0.0, y_bar = 0.0, x_sum = 0.0, y_sum = 0.0, z_sum = 0.0, z_bar = 0.0;
        arma::mat Z(n, 4);

        for (int k=0; k<n; k++){ // loop through points in cluster
            x_sum += clusters.at(j).at(k).x;
            y_sum += clusters.at(j).at(k).y;
            z_sum += std::pow(clusters.at(j).at(k).x, 2) + std::pow(clusters.at(j).at(k).y, 2);
        }

        // Find averages
        x_bar = x_sum / n ;
        y_bar = y_sum / n ;
        z_bar = z_sum / n ;

        for (int l=0; l<n; l++){ // loop through points in cluster again
            double x_i = clusters.at(j).at(l).x - x_bar;
            double y_i = clusters.at(j).at(l).y - y_bar;
            Z(l,0) = std::pow(x_i, 2) + std::pow(y_i, 2);
            Z(l,1) = x_i;
            Z(l,2) = y_i;
            Z(l,3) = 1;
        }

        arma::mat M = (1/n) * Z.t() * Z;
        arma::mat H = arma::eye(4,4);
        H(0,0) = 8*z_bar;
        H(0,3) = 2;
        H(3,0) = 2;
        H(3,3) = 0;
        arma::mat H_inv = arma::eye(4,4);
        H_inv(0,0) = 0;
        H_inv(0,3) = 0.5;
        H_inv(3,0) = 0.5;
        H_inv(3,3) = -2*z_bar;

        H_inv.print("H_inv");
        H.print("H");
        Z.print("Z");






        
    }

}


int main(int argc, char * argv[]){

    ros::init(argc, argv, "landmarks");
    ros::NodeHandle n;

    frequency = 500;
    ros::Rate r(frequency);
    n.param("max_lidar_range", max_range, 3.5);
    n.param("min_lidar_range", min_range, 0.120);

    ros::Subscriber laser_scan_sub = n.subscribe("/laser_data", 10, laser_scan_callback);

    while(ros::ok()){

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
