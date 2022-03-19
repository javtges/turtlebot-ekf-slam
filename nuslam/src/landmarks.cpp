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

/// \file
/// \brief Determines where the landmarks are from the lidar data, and publishes them accordingly.
///
/// PARAMETERS:
///     /nusim/frequency (int): Frequency of the node in Hz
///     /min_lidar_range (float): Minimum lidar range in meters
///     /max_lidar_range (float): Maximum lidar range in meters
/// PUBLISHES:
///     /fake_sensor (nav_msgs::MarkerArray): The markers found by the lidar data.
/// SUBSCRIBES:
///     /laser_scan (sensor_msgs::LaserScan): The raw ranges and bearings of the lidar data.


static int frequency;
static double max_range, min_range;
static visualization_msgs::MarkerArray ma;

static ros::Publisher marker_pub;

void laser_scan_callback(const sensor_msgs::LaserScan & msg){
    // ROS_WARN_STREAM("AAA");

    visualization_msgs::MarkerArray maTemp;
    int n_circles = 0;

    int length = msg.ranges.size();
    double threshold = 0.05;
    float range = 0.0, prev_range = 0.0;
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
            // ROS_WARN_STREAM(i << " " << range << " " << mx << " " << my);
            
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
                    // ROS_ERROR_STREAM("new cluster!");
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
    if (std::abs(first_distance - range) < threshold){
        clusters.at(0).insert( clusters.at(0).end(), clusters.at(n_clusters-1).begin(), clusters.at(n_clusters-1).end() );
        clusters.pop_back(); // Removes last cluster
        // ROS_WARN_STREAM("overlap clusters, " << clusters.at(0).size());
    }
    // ROS_WARN_STREAM("LOOPING THROUGH CLUSTERS " << clusters.size());

    for (int j=0; j<(int)clusters.size(); j++){ // loop through clusters
        int n = clusters.at(j).size();
        if (n > 4){    // We need at least 4 points to say it's a circle or not (and also to not crash the program)
            double x_bar = 0.0, y_bar = 0.0, x_sum = 0.0, y_sum = 0.0, z_sum = 0.0, z_bar = 0.0;
            double center_x, center_y, R;
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

            // H_inv.print("H_inv");
            // H.print("H");
            // Z.print("Z");

            // SINGULAR VALUE DECOMPOSITION -----------------------

            arma::mat U;
            arma::colvec sig_vec;
            arma::mat V;
            arma::colvec A(4);
            arma::colvec A_star(4);

            arma::svd(U, sig_vec, V, Z);

            // U.print("U");
            // sig_vec.print("sigma");
            // V.print("V");
            arma::mat Sigma = arma::diagmat(sig_vec);

            double min_sigma = sig_vec.min();

            if (min_sigma < 1.0e-12){
                A = V.col(3);
            }
            else{
                arma::mat Y = V * Sigma * V.t();
                arma::mat Q = Y * arma::inv(H) * Y;

                // Y.print("Y");
                // Q.print("Q");

                arma::mat eigenvectors;
                arma::colvec eigenvalues;

                arma::eig_sym(eigenvalues, eigenvectors, Q);

                // eigenvalues.print("eigenvalues");
                // eigenvectors.print("eigenvectors");

                double smallest_positive = 0.0;

                for(int val=0; val<=3; val++){
                    if(eigenvalues(val) > 0 ){
                        smallest_positive = eigenvalues(val);
                        A_star = eigenvectors.col(val);
                        // ROS_WARN_STREAM("smallest positive " << smallest_positive);
                        break;
                    }
                }

                A = arma::inv(Y) * A_star;

            } // end else (min_sigma > 10e-12)

            // Find location of the center and the radius of the circle

            // A.print("A");
            center_x = (-A(1) / (2*A(0))) + x_bar;
            center_y = (-A(2) / (2*A(0))) + y_bar;
            R = std::sqrt( (std::pow(A(1),2) + std::pow(A(2),2) - (4 * A(0) * A(3)) ) / (4 * std::pow(A(0),2)) );

            // ROS_WARN_STREAM("circle center " << center_x << " " << center_y << " " << R);

            // Now, determine if it's actually a circle or not

            turtlelib::Vector2D P1, P2, P;
            double P1_P, P_P2, P1_P2, angle_mean, angle_stdev;
            P1 = clusters.at(j).at(0);
            P1 = clusters.at(j).at(n-1);
            arma::colvec angles(n-2); // We'll store all the angles in here
            P1_P2 = std::sqrt( std::pow( (P1.x - P2.x), 2) + std::pow( (P1.y - P2.y) , 2) );

            for(int point = 1; point < n-1; point++){ // All the other points in the cluster are considered "P".
                P = clusters.at(j).at(point);
                P1_P = std::sqrt( std::pow( (P1.x - P.x), 2) + std::pow( (P1.y - P.y) , 2) );
                P_P2 = std::sqrt( std::pow( (P2.x - P.x), 2) + std::pow( (P2.y - P.y) , 2) );

                double numerator = std::pow(P1_P,2) + std::pow(P_P2,2) - std::pow(P1_P2,2);
                double denom = 2*(P1_P * P_P2);

                angles(point-1) = std::acos( (numerator/denom) );
            }

            // angles.print("angles?");
            angle_mean = arma::mean(angles);
            angle_stdev = arma::stddev(angles);
            // ROS_WARN_STREAM("Angle mean, stdev "<< angle_mean <<" " << angle_stdev);

            if ( (angle_mean < 2.4) && (angle_mean > 1.6) && (angle_stdev < 0.35 ) && (R > 0.02) && (R < 2.0) ){
                // ROS_ERROR_STREAM("A circle! " << j);

                visualization_msgs::Marker mark;
                mark.header.frame_id = "green-base_footprint";
                mark.header.stamp = ros::Time::now();
                mark.ns = "landmark_node";
                mark.id = j;
                mark.type = visualization_msgs::Marker::CYLINDER;
                mark.action = visualization_msgs::Marker::ADD;

                mark.pose.position.x = center_x;
                mark.pose.position.y = center_y;
                mark.pose.position.z = 0.125;
                mark.pose.orientation.x = 0.0;
                mark.pose.orientation.y = 0.0;
                mark.pose.orientation.z = 0.0;
                mark.pose.orientation.w = 1.0;

                mark.scale.x = R*2;
                mark.scale.y = R*2;
                mark.scale.z = 0.25;

                mark.color.r = 1.0;
                mark.color.g = 1.0;
                mark.color.b = 0.0;
                mark.color.a = 1.0;
                mark.lifetime = ros::Duration(0);

                maTemp.markers.push_back(mark);

                n_circles++;
            }
        }
    }// end of loop through clusters

    ma = maTemp;
    // ROS_WARN_STREAM("publishing circles qty " << n_circles);
    if (n_circles > 0){
        marker_pub.publish(ma);
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
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("fake_sensor",10);


    while(ros::ok()){

        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
