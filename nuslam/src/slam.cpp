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
/// \brief Handles the odometry to allow the blue turtlebot to move in simulation and real life.
///
/// PARAMETERS:
///     /nusim/frequency (parameter_type): description of the parameter
///     /nusim/frequency/x0 (double): The starting x coordinate of the turtlebot.
///     /nusim/frequency/y0 (double): The starting y coordinate of the turtlebot.
///     /nusim/frequency/theta0 (double): The starting orientation (yaw) of the turtlebot.
///     /odom_id (std::string): The name of the odometry frame, coincident with the world frame.
///     /body_ (std::string): The name of the body frame for the odometry.
///     /wheel_left (std::string): The name of the left wheel joint.
///     /wheel_right (std::string): The name of the right wheel joint.
/// PUBLISHES:
///     /odom (nav_msgs::Odometry): The odometry of the blue turtlebot. Publishes 500 times per second.
/// SUBSCRIBES:
///     /red/joint_states (sensor_msgs::JointStates): The joint states of the turtlebot.
/// SERVICES:
///     set_pose (nuturtle_control::SetPose): Changes the odometry so that the robot thinks it's at the given pose.


static std::string odom_frame, body_id, wheel_left, wheel_right;
static double x_0, y_0, theta_0;
static int frequency;
static int init_flag = 1;
static int EKF_SIZE = 25;
static turtlelib::Transform2D Tmb, Tob, Tmo, Tob_slam; //Map to Robot, Odom to Robot, Map to Odom
static turtlelib::DiffDrive drive;
static turtlelib::Q turtle_config;
static turtlelib::Phidot wheel_speeds;
static turtlelib::Twist2D twist;
static nuturtlebot_msgs::WheelCommands speeds;
static sensor_msgs::JointState joint_states;
static nav_msgs::Odometry odom;
static std::vector<double> positions, velocities, radii, x_locs, y_locs;
static std::vector<turtlelib::Vector2D> markers_in_map(3);
static nuslam::EKFilter kalman(EKF_SIZE);
static visualization_msgs::MarkerArray markers;
static long int counter=0;
static ros::Publisher odom_pub, marker_pub, path_pub;
static nav_msgs::Path green_path;
static geometry_msgs::PoseStamped green_pose;
static arma::mat Xi;


static int n_confirm = 0, n_prelim = 0;
static double mah_low = 0.1, mah_high = 0.5;
static arma::vec iterations(EKF_SIZE);
static arma::mat prelim_Xi = arma::mat(3 + (2*EKF_SIZE), 1, arma::fill::zeros);

/// \brief The callback function for the joint_state subscriber
/// Calculates the new turtlebot configuration, determines the instanteous twist, and begins populating the odometry message.
void joint_state_callback(const sensor_msgs::JointState::ConstPtr& msg){
    // Update internal odometry state

    turtlelib::Phidot currentSpeeds;
    turtlelib::Phi currentAngles, nextAngles;
    turtlelib::Q new_config;

    positions.resize(2);
    velocities.resize(2);
    positions = msg->position;
    velocities = msg->velocity;

    currentAngles.L = positions[0];
    currentAngles.R = positions[1];
    currentSpeeds.Ldot = velocities[0];
    currentSpeeds.Rdot = velocities[1];

    drive.setSpeeds(currentSpeeds);
    drive.setAngles(currentAngles);

    nextAngles.L = (currentSpeeds.Ldot/500) + currentAngles.L;
    nextAngles.R = (currentSpeeds.Rdot/500) + currentAngles.R;

    twist = drive.get_twist_from_angles(currentSpeeds);

    drive.forward_kinematics(nextAngles);

    odom.twist.twist.linear.x = twist.xdot;
    odom.twist.twist.linear.y = twist.ydot;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = twist.thetadot;

    turtle_config = drive.getConfig();
    Tob = turtlelib::Transform2D({turtle_config.x, turtle_config.y}, turtle_config.theta); // This is the transform from odom to the robot
    // Believe this is good


}

arma::vec find_mahalob(nuslam::EKFilter kalman, arma::mat prelim_Xi, turtlelib::Twist2D twist, double marker_x, double marker_y, int num_found){

    // ROS_WARN_STREAM("Test\r\n");
    // arma::mat Xi_current = kalman.get_Xi();
    arma::mat Xi_current = prelim_Xi;
    // Xi_current.print("prelim xi");
    std::cout << Xi_current << "\r\n";
    // arma::mat H_current = kalman.get_H();
    arma::mat Sigma_current = kalman.get_Sigma();
    arma::mat R = kalman.get_R();
    arma::mat Q = kalman.get_Q();

    int num_markers = kalman.get_n();
    arma::mat psi;

    arma::vec output(num_found, arma::fill::zeros);

    for (int p=0; p<num_found; p++){ //loop through the number of preliminary markers we have

        arma::mat z_hat = arma::mat(2,1);
        arma::mat z = arma::mat(2,1);
        
        double deltaX = Xi_current((2*p)+3,0) - Xi_current(1,0);
        double deltaY = Xi_current((2*p)+4,0) - Xi_current(2,0);
        double d = std::pow(deltaX,2) + std::pow(deltaY,2);

        double rad = std::sqrt( std::pow(deltaX, 2) + std::pow(deltaY, 2) ); //this is r_j
        double angle = std::atan2( deltaY, deltaX) - Xi_current(0,0); // This is phi_j
        angle = turtlelib::normalize_angle(angle);

        z_hat(0,0) = rad;
        z_hat(1,0) = angle;

        double z_rad = std::sqrt( std::pow(marker_x, 2) + std::pow(marker_y, 2) ); //this is z_j
        double z_angle = std::atan2( marker_y, marker_x ); // This is phi_j
        z_angle = turtlelib::normalize_angle(z_angle);
        
        z(0,0) = z_rad;
        z(1,0) = z_angle;

        arma::mat diff(2,1);

        diff = z - z_hat;

        arma::mat H = arma::mat(2,3+2*num_markers, arma::fill::zeros);

        H(0,0) = 0;
        H(0,1) = -deltaX / std::sqrt(d);
        H(0,2) = -deltaY / std::sqrt(d);
        H(1,0) = -1;
        H(1,1) = deltaY / d;
        H(1,2) = -deltaX / d;

        H(0,2*p+3) = deltaX / std::sqrt(d);
        H(0,2*p+4) = deltaY / std::sqrt(d);
        H(1,2*p+3) = -deltaY / d;
        H(1,2*p+4) = deltaX / d;

        // H.print("H matrix");

        arma::mat At_eye = arma::eye(3+2*num_markers, 3+2*num_markers);
        arma::mat At = arma::mat(3+2*num_markers, 3+2*num_markers, arma::fill::zeros);
        arma::mat A = arma::mat(3+2*num_markers, 3+2*num_markers, arma::fill::zeros);
        arma::mat K = arma::mat(3+2*num_markers, 3+2*num_markers, arma::fill::zeros);

        if(turtlelib::almost_equal(twist.thetadot, 0.0)){
            // If almost zero rotation:
            At(1,0) = -1*(twist.xdot) * std::sin(Xi_current(0,0));
            At(2,0) = (twist.xdot) * std::cos(Xi_current(0,0));
        }

        else{
            // Xi(0,0) += twist.thetadot;
            At(1,0) = ((-twist.xdot / twist.thetadot) * std::cos(Xi_current(0,0))) + ((twist.xdot / twist.thetadot) * std::cos( turtlelib::normalize_angle(Xi_current(0,0) + (twist.thetadot)) ));
            At(2,0) = ((-twist.xdot / twist.thetadot) * std::sin(Xi_current(0,0))) + ((twist.xdot / twist.thetadot) * std::sin( turtlelib::normalize_angle(Xi_current(0,0) + (twist.thetadot)) ));
        
        }

        A = At_eye + At;
        Sigma_current = (A * Sigma_current * A.t()) + Q;
        // Sigma_current.print("Calculated Sigma with A");
        K = (Sigma_current * H.t()) * arma::inv((H * Sigma_current * H.t()) + R);
        
        arma::mat KH = K*H;
        arma::mat I = arma::eye(arma::size(KH));
        Sigma_current = (I - KH) * Sigma_current;


        psi = (H * Sigma_current * H.t()) + R;

        arma::mat dk = (diff.t() * arma::inv(psi)) * diff;

        // ROS_ERROR_STREAM("marker " << p << " " << marker_x << " " << marker_y << " " << ", MAHOB DISTANCE " << dk(0) << "\r\n");

        output(p) = dk(0);

    }

    return output;
}

arma::vec find_euc(nuslam::EKFilter, arma::mat Xi, double marker_x, double marker_y, int num_found){

    arma::vec output(num_found, arma::fill::zeros);


    for (int p=0; p<num_found; p++){

        arma::mat z_hat = arma::mat(2,1);
        arma::mat z = arma::mat(2,1);
        
        double deltaX = Xi((2*p)+3,0) - Xi(1,0);
        double deltaY = Xi((2*p)+4,0) - Xi(2,0);
        double d = std::pow(deltaX,2) + std::pow(deltaY,2);

        double rad = std::sqrt( std::pow(deltaX, 2) + std::pow(deltaY, 2) ); //this is r_j
        double angle = std::atan2( deltaY, deltaX) - Xi(0,0); // This is phi_j
        angle = turtlelib::normalize_angle(angle);

        z_hat(0,0) = rad;
        z_hat(1,0) = angle;

        double z_rad = std::sqrt( std::pow(marker_x, 2) + std::pow(marker_y, 2) ); //this is z_j
        double z_angle = std::atan2( marker_y, marker_x ); // This is phi_j
        z_angle = turtlelib::normalize_angle(z_angle);

        double mark_map_x, mark_map_y;
        mark_map_x = Xi(1,0) + z_rad * (std::cos(z_angle + Xi(0,0)));
        mark_map_y = Xi(2,0) + z_rad * (std::sin(z_angle + Xi(0,0)));


        // deltaX = Xi(1,0) + (rad * std::cos(angle));
        // deltaY = Xi(2,0) + (rad * std::sin(angle));

        // ROS_ERROR_STREAM("zhat " << deltaX << " " << deltaY << " " << "z " << mark_map_x << " " << mark_map_y << "\r\n");
        
        z(0,0) = z_rad;
        z(1,0) = z_angle;

        arma::mat diff(2,1);

        diff = z - z_hat;

        double dist = std::sqrt( std::pow((mark_map_x - Xi((2*p)+3,0)), 2) + std::pow((mark_map_y - Xi((2*p)+4,0)), 2) );

        output(p) = dist;

        // ROS_ERROR_STREAM("marker " << p << " " << mark_map_x << " " << mark_map_y << " " << ", EUC DISTANCE " << dist << "\r\n");


    }

    return output;
}


void fake_sensor_callback(const visualization_msgs::MarkerArray & msg){

        // IN SUBSCRIBER CALLBACK FOR FAKE_SENSOR:
        // LOOP THROUGH THE LANDMARKS
        // IF NOT INIT YET
            // kalman.init_landmarks(marker_id, double x, double y) <- make sure to do this all in the map frame
            // ^^ DONE, BUT WITH TRANSFORMS ISSUES
            // kalman.init_Q; -> DONE
        // THE ORDER OF THE SLAM
            // kalman.predict(twist,time);
            // kalman.UpdateMeasurement(marker j) <- use the current Xi and robot position to find Z_hat
            // kalman.ComputeKalmanGains()
            // kalman.UpdatePosState(x_loc, y_loc) <- find Z from the fake_sensor measurement
            // kalman.UpdateCovariance()
            // Use Xi to determine green robot's location
    int num_markers = msg.markers.size();
    int marker_index = 0;
    arma::mat test_sig = kalman.get_Sigma();

    for (int i=0; i<num_markers; i++){
        
        if (n_confirm == 0){
            kalman.init_landmarks(n_confirm, msg.markers[i].pose.position.x, msg.markers[i].pose.position.y);
            n_confirm++;
            continue;
        }

        // Find all of the mah_distances, using the preliminary Xi matrix
        
        arma::vec euc_distances = find_euc(kalman, kalman.get_Xi(), msg.markers[i].pose.position.x, msg.markers[i].pose.position.y, n_confirm);

        int argmin_distances = euc_distances.index_min(); // The index of the closest marker

        double min_distance = euc_distances.at(argmin_distances); // The distance of the closest marker

        // ----------------------------------------------------

        if( min_distance < mah_low ){ // If distance is lower than the threshold, it's already in prelim_Xi
            
            marker_index = argmin_distances;
        }
        else if ( min_distance > mah_high ){ // If it's higher than the threshold, it's a NEW landmark
            
            // Set the Xi state vector to include the new potential landmark
            kalman.init_landmarks(n_confirm, msg.markers[i].pose.position.x, msg.markers[i].pose.position.y);
            marker_index = n_confirm;
            n_confirm++; 

        }
        else{ // If the min_distance is between the two thresholds, throw it out
            continue;
        }

        kalman.Predict(twist);
        kalman.UpdateMeasurement(marker_index); // Update the measurement of the index of the closest marker
        kalman.ComputeKalmanGains();
        kalman.UpdatePosState(msg.markers[i].pose.position.x, msg.markers[i].pose.position.y);
        kalman.UpdateCovariance();
        iterations(argmin_distances) += 1;

    }

    init_flag = 0;    
    
}

visualization_msgs::MarkerArray markers_from_slam(){

    arma::mat Xi = kalman.get_Xi();

    int l = 3;
    visualization_msgs::MarkerArray maTemp;
    maTemp.markers.resize(l);

    // std::normal_distribution<> d(0, basic_sensor_variance); /// Args are mean,variance

    for (int i=0; i<l; i++){

        double markerX = Xi((2*i)+3, 0);
        double markerY = Xi((2*i)+4, 0);
        double mag = std::sqrt(markerX*markerX + markerY*markerY);

        maTemp.markers[i].header.frame_id = "map";
        maTemp.markers[i].header.stamp = ros::Time::now();
        maTemp.markers[i].ns = "slam_node";
        maTemp.markers[i].id = i;
        maTemp.markers[i].type = visualization_msgs::Marker::CYLINDER;
        if (mag > 3.5){
            maTemp.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        else{
            maTemp.markers[i].action = visualization_msgs::Marker::ADD;
            // ROS_ERROR_STREAM("ADDING");
        }
        maTemp.markers[i].pose.position.x = markerX;
        maTemp.markers[i].pose.position.z = 0.125;
        maTemp.markers[i].pose.position.y = markerY;
        maTemp.markers[i].pose.orientation.x = 0.0;
        maTemp.markers[i].pose.orientation.y = 0.0;
        maTemp.markers[i].pose.orientation.z = 0.0;
        maTemp.markers[i].pose.orientation.w = 1.0;

        maTemp.markers[i].scale.x = 0.038*2;
        maTemp.markers[i].scale.y = 0.038*2;
        maTemp.markers[i].scale.z = 0.25;

        maTemp.markers[i].color.r = 1.0;
        maTemp.markers[i].color.g = 1.0;
        maTemp.markers[i].color.b = 0.0;
        maTemp.markers[i].color.a = 1.0;
        maTemp.markers[i].lifetime = ros::Duration(0);
    }
    return maTemp;

}

void timerCallback(const ros::TimerEvent&){
    odom_pub.publish(odom);
    markers = markers_from_slam();
    marker_pub.publish(markers);

    green_path.header.stamp = ros::Time::now();
    green_path.header.frame_id = "world";
    
    green_pose.header.stamp = ros::Time::now();
    green_pose.header.frame_id = "world";
    green_pose.pose.position.x = Xi(1,0);
    green_pose.pose.position.y = Xi(2,0);

    green_path.poses.push_back(green_pose);

    path_pub.publish(green_path);
}


/// The main function and loop
int main(int argc, char * argv[])
{
    /// Initalize the node and the nodehandler, one is public and one is private.
    ros::init(argc, argv, "slam");

    ros::NodeHandle nh("~");
    ros::NodeHandle n;

    /// Gets the required values from the parameter server. Default values are provided for frequency, x0, y0, and theta0.
    frequency = 500;
    n.param("/nusim/x0", x_0, 0.0);
    n.param("/nusim/y0", y_0, 0.0);
    n.param("/nusim/theta0", theta_0, 0.0);
    n.param("odom_id", odom_frame, std::string("odom"));
    n.getParam("/nusim/radii", radii);
    n.getParam("/nusim/x_pos", x_locs);
    n.getParam("/nusim/y_pos", y_locs);

    ros::Rate r(frequency);

    turtlelib::Q initial_config;
    initial_config.theta = theta_0;
    initial_config.x = x_0;
    initial_config.y = y_0;
    
    if (!n.getParam("body_id",body_id)){
        ROS_ERROR_STREAM("Body ID frame not found!");
        ros::shutdown();
    }
    body_id = "green-base_footprint";

    /// Setting up the looping rate and the required subscribers.
    
    ros::Subscriber joint_state_sub = n.subscribe("/red/joint_states",10, joint_state_callback);
    ros::Subscriber fake_sensor_sub = n.subscribe("/fake_sensor", 10, fake_sensor_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom",100);
    marker_pub = n.advertise<visualization_msgs::MarkerArray>("slam_obstacles",10);
    ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback);
    path_pub = n.advertise<nav_msgs::Path>("green_path", 100);

    // ros::Timer timer = n.createTimer(ros::Duration(1/5), timerCallback);

    // ros::ServiceServer setPoseService = nh.advertiseService("set_pose", set_poseCallback);

    drive.setConfig(initial_config);
    Tmb = turtlelib::Transform2D({initial_config.x, initial_config.y}, initial_config.theta); //Perhaps this should be set to (0,0)...
    kalman.EKFilter_init(initial_config);
    kalman.init_Q_R(1, 0.0002);

    prelim_Xi(0,0) = initial_config.theta;
    prelim_Xi(1,0) = initial_config.x;
    prelim_Xi(2,0) = initial_config.y;

    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped_ob;
    geometry_msgs::TransformStamped transformStamped_mo;

    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {

        // Get the current config of the turtlebot
        turtle_config = drive.getConfig();
        // ROS_WARN_STREAM("got config");
        Xi = kalman.get_Xi();
        turtlelib::Vector2D xy{Xi(1,0), Xi(2,0)};
        Tmb = turtlelib::Transform2D(xy, Xi(0,0)); //The transform from the map to the robot - SLAM

        Tmo = Tmb * Tob.inv();

        transformStamped_mo.header.stamp = ros::Time::now();
        transformStamped_mo.header.frame_id = "map";
        transformStamped_mo.child_frame_id = odom_frame;
        transformStamped_mo.transform.translation.x = Tmo.translation().x;
        transformStamped_mo.transform.translation.y = Tmo.translation().y;
        transformStamped_mo.transform.translation.z = 0;
        tf2::Quaternion qm;
        qm.setRPY(0, 0, Tmo.rotation());
        transformStamped_mo.transform.rotation.x = qm.x();
        transformStamped_mo.transform.rotation.y = qm.y();
        transformStamped_mo.transform.rotation.z = qm.z();
        transformStamped_mo.transform.rotation.w = qm.w();

        br.sendTransform(transformStamped_mo);

        /// Make the transform from the odom frame to the GREEN body frame - needs to be Tob
        transformStamped_ob.header.stamp = ros::Time::now();
        transformStamped_ob.header.frame_id = odom_frame;
        transformStamped_ob.child_frame_id = "green-base_footprint";
        transformStamped_ob.transform.translation.x =  Tob.translation().x;
        transformStamped_ob.transform.translation.y = Tob.translation().y;
        transformStamped_ob.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, Tob.rotation());
        transformStamped_ob.transform.rotation.x = q.x();
        transformStamped_ob.transform.rotation.y = q.y();
        transformStamped_ob.transform.rotation.z = q.z();
        transformStamped_ob.transform.rotation.w = q.w();

        br.sendTransform(transformStamped_ob);

        // Make and publish the odometry message using the current green turtle configuration
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = odom_frame;
        odom.child_frame_id = body_id;
        odom.pose.pose.position.x = Tob.translation().x;
        odom.pose.pose.position.y = Tob.translation().y;
        odom.pose.pose.position.z = 0;
        tf2::Quaternion quat;
        quat.setRPY(0,0,Tob.rotation());
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();


        // Tmo = Tmb; // JUST FOR TESTING
        // Make and publish the transform from map to odom in order to complete the tree

        counter++;
        ros::spinOnce();
        r.sleep();
    }

    return 0;   
}