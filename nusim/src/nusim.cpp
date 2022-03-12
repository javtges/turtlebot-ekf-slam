#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <cmath>
#include <random>
#include <string>
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include "nusim/teleport.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nuturtlebot_msgs/SensorData.h>
#include <nuturtlebot_msgs/WheelCommands.h>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"


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
///     ~motor_cmd_to_radsec (double): The motor command to radians/second conversion factor.
///     ~encoder_ticks_to_rad (double): The encoder ticks to radians conversion factor.
///     ~motor_cmd_max (std::vector<double>): The maximum and minimum values of dynamixel ticks.
/// PUBLISHES:
///     /nusim/obstacles (visualization_msgs::MarkerArray): Publishes and displays cylindrical markers in rviz that represents obstacles for the turtlebot. This is a latched publisher that publishes once with the markers being permenant.
///     /nusim/timestep (std_msgs::UInt64): Increments a 64-bit int that acts as the timestamp for the simulation. This timestamp updates 500 times per second.
///     /nusim/walls (visualization_msgs::MarkerArray): Publishes and displays rectangluar markers in rviz that represents walls for the turtlebot. This is a latched publisher that publishes once with the markers being permenant.
///     /sensor_data (nuturtlebot_msgs::SensorData): Publishes encoder data according to wheel speeds and angles.
/// SUBSCRIBES:
///     /wheel_cmd (nuturtlebot_msgs::WheelCommands): The wheel velocities in dynamixel ticks.
/// SERVICES:
///     reset (std_srvs::Empty): teleports the turtlebot back to its starting position. additionally, resets the timestamp counter.
///     teleport (nusim::teleport): teleports the turtlebot to a given x, y, and theta pose.


static std_msgs::UInt64 ts;
static sensor_msgs::JointState jointState;
static geometry_msgs::TransformStamped transformStamped;
static visualization_msgs::MarkerArray ma, walls, sim_obstacles;
static nuturtlebot_msgs::SensorData sensor_data;
static double x_length, y_length, slip_min, slip_max, rand_slip, basic_sensor_variance, max_range, collision_radius;
static double x_0, y_0, theta_0;
static double left_velocity, right_velocity;
static double motor_cmd_to_radsec, encoder_ticks_to_rad;
static double min_lidar_range, max_lidar_range, min_lidar_angle, max_lidar_angle, lidar_angle_resolution, lidar_range_resolution, lidar_noise_mean, lidar_noise_stddev;
static int num_lidar_samples;
static turtlelib::DiffDrive drive;
static std::vector<double> radii, x_locs, y_locs, motor_cmd_max;
static std::vector<turtlelib::Vector2D> obstacles;
static turtlelib::Phi wheel_angles, wheel_angles_old;
static turtlelib::Phidot wheel_speeds;
static turtlelib::Q turtle_config, previous_config;
static sensor_msgs::LaserScan laserScan;
static ros::Publisher fake_sensor_pub, laser_scan_pub, path_pub;
static geometry_msgs::PoseStamped red_pose;
static nav_msgs::Path red_path;


/// \brief The callback function for the reset service. Resets the timestamp counter and teleports the turtlebot back to its starting pose.
/// \param &Request - the inputs to the service. For this type there are none.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully

std::mt19937 & get_random()
{
    // static variables inside a function are created once and persist for the remainder of the program
    static std::random_device rd{}; 
    static std::mt19937 mt{rd()};
    // we return a reference to the pseudo-random number genrator object. This is always the
    // same object every time get_random is called
    return mt;
}


bool resetCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &){
    ts.data = 0;
    turtle_config.x = x_0;
    turtle_config.y = y_0;
    turtle_config.theta = theta_0;

    drive.setConfig(turtle_config);

    return true;
}

/// \brief The callback function for the teleport service. Teleports the turtlebot to a given pose.
/// \param &Request - the inputs to the service. For this type there is an x, y, and theta float64 input.
/// \param &Response - the outputs of the service. For this type there are none.
/// returns true if executed successfully
bool teleportCallback(nusim::teleport::Request &Request, nusim::teleport::Response &){
    turtle_config.x = Request.x;
    turtle_config.y = Request.y;
    turtle_config.theta = Request.theta;

    drive.setConfig(turtle_config);

    return true;
}

/// \brief The callback function for the wheel_command subscriber
/// Sets the wheel_speeds variable to the current wheel speeds in radians/second
void wheelCallback(const nuturtlebot_msgs::WheelCommands::ConstPtr& msg){

    left_velocity = msg->left_velocity;
    right_velocity = msg->right_velocity;
    
    wheel_speeds.Ldot = left_velocity * motor_cmd_to_radsec;
    wheel_speeds.Rdot = right_velocity * motor_cmd_to_radsec;
    // ROS_ERROR_STREAM(wheel_speeds.Ldot);

    if (wheel_speeds.Ldot != 0.0){
        std::normal_distribution<> d(0, 0.0); /// Args are mean,variance
        wheel_speeds.Ldot += 0; //d(get_random());
        // ROS_ERROR_STREAM(wheel_speeds.Ldot);
    }
    if (wheel_speeds.Rdot != 0.0){
        std::normal_distribution<> d(0, 0.0); /// Args are mean,variance
        wheel_speeds.Rdot += 0; //d(get_random());
    }

    

}

/// \brief Converts a radians measurement to turtlebot3 wheel encoder ticks.
/// \param radians - the angle in radians
/// returns the corresponding amount of encoder ticks
int toEncoderTicks(double radians){
    return (int)(radians/encoder_ticks_to_rad) % 4096;
}


/// \brief Converts an encoder ticks measurement to radians.
/// \param ticks - the angle in encoder ticks
/// returns the corresponding amount of radians
double toRadians(int ticks){
    return ticks * encoder_ticks_to_rad;
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
        maTemp.markers[i].id = i+5;
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


/// \brief Populates a MarkerArray message with the walls in the parameter server as specified by the yaml.
/// \param x_length - the x (interior) dimension of the turtlebot arena
/// \param y_length - the y (interior) dimension of the turtlebot arena
/// returns the resulting MarkerArray ROS message.
visualization_msgs::MarkerArray addWalls(double x_length, double y_length){

    int l = 4;
    double t = 0.1;
    visualization_msgs::MarkerArray maTemp;
    maTemp.markers.resize(4);

    for (int i=0; i<l; i++){
        
        maTemp.markers[i].header.frame_id = "world";
        maTemp.markers[i].header.stamp = ros::Time::now();
        maTemp.markers[i].ns = "walls";
        maTemp.markers[i].id = i;
        maTemp.markers[i].type = visualization_msgs::Marker::CUBE;
        maTemp.markers[i].action = visualization_msgs::Marker::ADD;
        maTemp.markers[i].color.r = 1.0;
        maTemp.markers[i].color.g = 0.0;
        maTemp.markers[i].color.b = 0.0;
        maTemp.markers[i].color.a = 1.0;
        maTemp.markers[i].lifetime = ros::Duration(0);
    }

    // Wall 1
    maTemp.markers[0].pose.position.x = (x_length/2) + (t/2);
    maTemp.markers[0].pose.position.z = 0.25/2;
    maTemp.markers[0].pose.position.y = 0;
    maTemp.markers[0].pose.orientation.x = 0.0;
    maTemp.markers[0].pose.orientation.y = 0.0;
    maTemp.markers[0].pose.orientation.z = 0.0;
    maTemp.markers[0].pose.orientation.w = 1.0;

    maTemp.markers[0].scale.x = 0.1;
    maTemp.markers[0].scale.y = y_length;
    maTemp.markers[0].scale.z = 0.25;
    
    // Wall 2
    maTemp.markers[1].pose.position.x = 0;
    maTemp.markers[1].pose.position.z = 0.25/2;
    maTemp.markers[1].pose.position.y = -(y_length/2) - (t/2);
    maTemp.markers[1].pose.orientation.x = 0.0;
    maTemp.markers[1].pose.orientation.y = 0.0;
    maTemp.markers[1].pose.orientation.z = 0.0;
    maTemp.markers[1].pose.orientation.w = 1.0;

    maTemp.markers[1].scale.x = x_length+2*t;
    maTemp.markers[1].scale.y = 0.1;
    maTemp.markers[1].scale.z = 0.25;

    // Wall 3
    maTemp.markers[2].pose.position.x = -(x_length/2) - (t/2);
    maTemp.markers[2].pose.position.z = 0.25/2;
    maTemp.markers[2].pose.position.y = 0;
    maTemp.markers[2].pose.orientation.x = 0.0;
    maTemp.markers[2].pose.orientation.y = 0.0;
    maTemp.markers[2].pose.orientation.z = 0.0;
    maTemp.markers[2].pose.orientation.w = 1.0;

    maTemp.markers[2].scale.x = 0.1;
    maTemp.markers[2].scale.y = y_length+2*t;
    maTemp.markers[2].scale.z = 0.25;

    // Wall 4
    maTemp.markers[3].pose.position.x = 0;
    maTemp.markers[3].pose.position.z = 0.25/2;
    maTemp.markers[3].pose.position.y = (y_length/2) + (t/2);
    maTemp.markers[3].pose.orientation.x = 0.0;
    maTemp.markers[3].pose.orientation.y = 0.0;
    maTemp.markers[3].pose.orientation.z = 0.0;
    maTemp.markers[3].pose.orientation.w = 1.0;

    maTemp.markers[3].scale.x = x_length;
    maTemp.markers[3].scale.y = 0.1;
    maTemp.markers[3].scale.z = 0.25;

    return maTemp;

}

visualization_msgs::MarkerArray simulateObstacles(std::vector<double> radii, std::vector<double> x_locs, std::vector<double> y_locs){

    int l = radii.size();
    visualization_msgs::MarkerArray maTemp;
    maTemp.markers.resize(l);

    turtlelib::Q current_config = drive.getConfig();
    turtlelib::Vector2D current_vec = {current_config.x, current_config.y};
    double current_theta = current_config.theta;
    turtlelib::Transform2D Twr(current_vec,current_theta);
    turtlelib::Transform2D Trw = Twr.inv(); //World in robot frame

    // turtlelib::Vector2D robot_to_world = Trw.translation();

    std::normal_distribution<> d(0, basic_sensor_variance); /// Args are mean,variance

    for (int i=0; i<l; i++){

        turtlelib::Vector2D markervec = {x_locs[i], y_locs[i]};
        turtlelib::Transform2D Twm(markervec);
        turtlelib::Transform2D Trm(0);
        Trm = Trw * Twm;
        turtlelib::Vector2D robot_to_marker = Trm.translation();

        double mag = std::sqrt(robot_to_marker.x*robot_to_marker.x + robot_to_marker.y*robot_to_marker.y);

        maTemp.markers[i].header.frame_id = "red-base_footprint";
        maTemp.markers[i].header.stamp = ros::Time::now();
        maTemp.markers[i].ns = "nusim_node";
        maTemp.markers[i].id = i;
        maTemp.markers[i].type = visualization_msgs::Marker::CYLINDER;
        if (mag > max_range){
            maTemp.markers[i].action = visualization_msgs::Marker::DELETE;
        }
        else{
            maTemp.markers[i].action = visualization_msgs::Marker::ADD;
            // ROS_ERROR_STREAM("ADDING");
        }
        maTemp.markers[i].pose.position.x = robot_to_marker.x + d(get_random());
        maTemp.markers[i].pose.position.z = 0.125;
        maTemp.markers[i].pose.position.y = robot_to_marker.y + d(get_random());
        maTemp.markers[i].pose.orientation.x = 0.0;
        maTemp.markers[i].pose.orientation.y = 0.0;
        maTemp.markers[i].pose.orientation.z = 0.0;
        maTemp.markers[i].pose.orientation.w = 1.0;

        maTemp.markers[i].scale.x = radii[i]*2;
        maTemp.markers[i].scale.y = radii[i]*2;
        maTemp.markers[i].scale.z = 0.25;

        maTemp.markers[i].color.r = 0.0;
        maTemp.markers[i].color.g = 1.0;
        maTemp.markers[i].color.b = 0.0;
        maTemp.markers[i].color.a = 1.0;
        maTemp.markers[i].lifetime = ros::Duration(0);

    }
    return maTemp;

}

double findDistance(auto x1, auto y1, auto x2, auto y2){
    return std::sqrt(std::pow(x2-x1,2) + std::pow(y2-y1,2));
}

int sign(auto dy){
    if (dy < 0){
        return -1;
    }
    else{
        return 1;
    }
}

void simulateLidar(){

    // ROS_ERROR_STREAM("Simulating Lidar");
    laserScan.header.frame_id = "red-base_scan";
    laserScan.header.stamp = ros::Time::now();
    laserScan.angle_min = 0.0; //min_lidar_angle;
    laserScan.angle_max = 6.28319; // max_lidar_angle;
    laserScan.angle_increment = turtlelib::deg2rad(1); // lidar_angle_resolution; //This is in degrees 
    laserScan.time_increment = 1/1800;
    laserScan.scan_time = 1/5;
    laserScan.range_min = 0.120; // min_lidar_range;
    laserScan.range_max = 3.5; // max_lidar_range;
    std::vector<float> ranges(num_lidar_samples,0.0);
    // ROS_ERROR_STREAM("made a new thing");
    
    laserScan.ranges = ranges;

    // int max_angle = (int)turtlelib::rad2deg(laserScan.angle_max);
    // int min_angle = (int)turtlelib::rad2deg(laserScan.angle_min);
    // for (int i=0; i<360; i++){
        // laserScan.ranges[i] = 3.4;
    // }

    // have too many points, need to filter out

    // Loop across angles
    for (int i=0; i<num_lidar_samples; i++){
        // ROS_ERROR_STREAM(i);
        turtlelib::Q current_config = drive.getConfig();
        turtlelib::Vector2D current_vec = {current_config.x, current_config.y};
        double current_theta = current_config.theta;
        // ROS_WARN("%f",current_theta);
        turtlelib::Transform2D Twb(current_vec,current_theta); // The robot in the world frame
        // turtlelib::Transform2D Tbw = Twb.inv(); // The world in the robot frame
        // laserScan.ranges[i] = 3;

        // turtlelib::Vector2D robot_to_world = Trw.translation(); // the robot to the world vector

        // Loop across obstacles
        int l = radii.size();
        for (int j=0; j<l; j++){

            turtlelib::Vector2D markervec = {x_locs[j], y_locs[j]};
            turtlelib::Transform2D Two(markervec); // The marker in the world frame
            turtlelib::Transform2D Tow = Two.inv(); // World in marker/obstacle frame
            turtlelib::Transform2D Tob(0), Tbo(0); // The robot in the obstacle frame
            
            Tob = Tow * Twb;
            Tbo = Tob.inv();

            int angle = i;
            double u_x=0, u_y=0;

            // Unit vector components for the direction
            u_x = std::cos(turtlelib::deg2rad(angle)) * laserScan.range_min;
            u_y = std::sin(turtlelib::deg2rad(angle)) * laserScan.range_min;
            // ROS_ERROR_STREAM(angle << " " << u_x << " " << u_y);

            double dx=0, dy=0, dr=0, D=0, x1=0, x2=0, y1=0, y2=0, delta=0;
            float distance = 0;
            int reflection = 0;
            
            turtlelib::Vector2D vec1, vec2;
            vec1.x = 0;
            vec1.y = 0;
            vec1 = Tob(vec1);

            vec2.x = u_x;
            vec2.y = u_y;
            vec2 = Tob(vec2);

            x1 = vec1.x;
            y1 = vec1.y;

            x2 = vec2.x;
            y2 = vec2.y;

            // ROS_ERROR_STREAM("robot x in marker: " << x1 << " robot y in marker: " << y1);
            // ROS_ERROR_STREAM("OTHER X: " << vec1.x << " OTHER Y: " << vec1.y);
            
            dx = x2-x1;
            dy = y2-y1;

            dr = std::sqrt((dx*dx) + (dy*dy));

            D = (x1*y2) - (x2*y1);

            delta = (radii[j]*radii[j]*dr*dr) - (D*D);

            if (delta >= 0){ // This means there is an intersection with the marker at this angle
                // Find where it intersects (both points), and keep the minimum one of the two
                // ROS_ERROR_STREAM("found an intersection at " << angle << " and marker " << j);
                double xint1=0, yint1=0, xint2=0, yint2=0, mag1=0, mag2=0;
                // laserScan.ranges[i] = 0.5;

                xint1 = ((D * dy) + (sign(dy) * dx*std::sqrt(delta))  )/(dr*dr);
                yint1 = ((-1 * D * dx) + (std::abs(dy) * std::sqrt(delta))  )/(dr*dr);

                xint2 = ((D * dy) - (sign(dy) * dx*std::sqrt(delta)) ) /(dr*dr);
                yint2 = ((-1 * D * dx) - (std::abs(dy) * std::sqrt(delta))  )/(dr*dr);

                // Need to find the distance from these points x1y1 x2y2 to the robot, and THEN find the magnitude!
                mag1 = findDistance(x1,y1,xint1,yint1);
                mag2 = findDistance(x1,y1,xint2,yint2);

                turtlelib::Vector2D int1, int2, intersection;
                double magnitude;
                int1.x = xint1; int1.y = yint1;
                int2.x = xint2; int2.y = yint2;
                
                // Find the minimum of the two magnitudes, record it and the corresponding intersection points with the circle.
                if (mag1 < mag2){
                    intersection = int1;
                    magnitude = mag1;
                }
                else{
                    intersection = int2;
                    magnitude = mag2;
                }

                // Sets a flag to remove infinite line intersections
                if (findDistance(x2,y2,intersection.x,intersection.y) > magnitude){
                    // ROS_WARN("marker %d distance %f at %d, REFLECTION",j, magnitude,i);
                    reflection = 1;
                }

                // Compare with other markers that may be colinear
                if (laserScan.ranges[i] > 0.0){
                    distance = std::min(laserScan.ranges[i], (float)magnitude);
                }
                // If there are no colinear markers
                else{
                    distance = (float)magnitude;
                }

                // Only set the point if it's not a reflection
                if (reflection == 0){
                    distance -= std::fmod(distance,lidar_range_resolution);
                    laserScan.ranges[i] = distance;
                    // ROS_WARN("marker %d distance %f at %d, ADDED POINT",j, laserScan.ranges[i],i);
                }
                
            }
            
            else{ // Delta is negative, so there's no intersection
            }
            // Remove values that are outside of acceptable ranges for the lidar
        }
        
        // If there's no populated laserScan input after looking at the markers
        if (laserScan.ranges[i] == 0.0){
            // Check for walls
            double u_x=0.0, u_y=0.0;
            float output = 0.0;
            int angle = i;
            // Unit vector components for the direction
            u_x = std::cos(turtlelib::deg2rad(angle) + current_theta) * laserScan.range_min;
            u_y = std::sin(turtlelib::deg2rad(angle) + current_theta) * laserScan.range_min;
            
            double x1=0, x2=0, y1=0, y2=0, x3=0, y3=0, x4=0, y4=0 ;
            std::vector<turtlelib::Vector2D> edges{{-x_length/2, -y_length/2}, {x_length/2, -y_length/2}, {x_length/2, y_length/2}, {-x_length/2, y_length/2}};
            std::vector<turtlelib::Vector2D> intersections{{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}};
            std::vector<float> distances(3);
            x1 = current_config.x;
            y1 = current_config.y;
            
            // There's going to be a problem here with rotations
            x2 = current_config.x + u_x;
            y2 = current_config.y + u_y;

            float den = 0.0;
            float Px=0.0, Py=0.0;
            // Find the points for line-line intersection. https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
            for(int k=0; k<=3; k++){
                x3 = edges[k].x;
                y3 = edges[k].y;
                if (k==3){
                    x4 = edges[0].x;
                    y4 = edges[0].y;
                }
                else{
                    x4 = edges[k+1].x;
                    y4 = edges[k+1].y;
                }

                den = ((x1-x2)*(y3-y4)) - ((y1-y2)*(x3-x4));
                // Good above here

                // ROS_ERROR_STREAM("angle " << i << " denominator " << den);
                // ROS_ERROR_STREAM("x1: " << x1 << " y1: " << y1 << " x2: " << x2 << " y2: " << y2 << " x3: " << x3 << " y3: " << y3 << " x4: " << x4 << " y4: " << y4);
                
                // Nonzero denominator means that there's an intersection
                // Vertical walls - works
                //if (den != 0.0 && (k%2) == 1){
                if (x3 == x4){
                    Px = (((x1*y2) - (y1*x2))*(x3-x4) - (x1-x2)*((x3*y4) - (y3*x4))) / den ;
                    Py = (((x1*y2) - (y1*x2))*(y3-y4) - (y1-y2)*((x3*y4) - (y3*x4))) / den ;
                    intersections[k].x = Px;
                    intersections[k].y = Py;
                    // Px = ((x1*y2 - y1*x2)*(x4-x3) - (x1-x2)*(x4*x3 - y3*x3)) / den ;
                    // Py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x4*x3 - y3*x3)) / den ;

                    // ROS_ERROR_STREAM("Horizontal Wall Px: " << Px << " Py: " << Py);
                    // if (std::abs(Px) < 5 && std::abs(Py) < 5){
                        // ROS_ERROR_STREAM("Px: " << Px << " Py: " << Py);
                    // }
                }
                //}
                // Horizontal walls - currently don't work
                //else if (den != 0.0 && (k%2) == 0){
                if (y3 == y4){
                    //  Px = ((x1*y2 - y1*x2)*(x4-x3) - (x1-x2)*(x4*x3 - y3*x3)) / den ;
                    //  Py = ((x1*y2 - y1*x2)*(y3-y4) - (y1-y2)*(x4*x3 - y3*x3)) / den ;
                    Px = (((x1*y2) - (y1*x2))*(x3-x4) - (x1-x2)*((x3*y4) - (y3*x4))) / den ;
                    Py = (((x1*y2) - (y1*x2))*(y3-y4) - (y1-y2)*((x3*y4) - (y3*x4))) / den ;
                    intersections[k].x = Px;
                    intersections[k].y = Py;
                    // ROS_ERROR_STREAM("Vertical Wall Px: " << Px << " Py: " << Py);
                }
                //}
                // If no intersection, set the "intersection" equal to something beyond the limits anyways
            }

            for(int ints=0; ints<=3; ints++){
                // If the absolute values of the intersection points are both within the range of the arena
                if( std::abs(intersections[ints].x) <= x_length/2 && std::abs(intersections[ints].y) <= y_length/2 ){
                    // Find distance from turtlebot to wall, Check for reflections
                    float robot_to_wall = 0, x2_to_wall = 0;
                    robot_to_wall = findDistance(x1, y1, intersections[ints].x, intersections[ints].y);
                    x2_to_wall = findDistance(x2, y2, intersections[ints].x, intersections[ints].y);
                    // ROS_WARN("found an intersection with angle %d, at points %f x and %f y, distance %f", i, intersections[ints].x, intersections[ints].y, robot_to_wall);
                    
                    // Check for reflections
                    if (robot_to_wall > x2_to_wall){ // If this is NOT a reflection
                    //    ROS_WARN("%f is greater than %f, not a reflection", robot_to_wall, x2_to_wall);
                       //distances.push_back(robot_to_wall);
                       output = robot_to_wall;
                    }
                    // If no reflection, set the laserdata to be equal to the minimum of the two distances
                }
                else{

                }
            }
            // float output = *std::min_element(distances.begin(), distances.end());
            // ROS_WARN("%f", output);
            laserScan.ranges[i] = output;

        }
    }
}

void timerCallback(const ros::TimerEvent&){
    ma = simulateObstacles(radii, x_locs, y_locs);
    fake_sensor_pub.publish(ma);

    simulateLidar();
    laser_scan_pub.publish(laserScan);


    turtle_config = drive.getConfig();

    red_path.header.stamp = ros::Time::now();
    red_path.header.frame_id = "world";
    
    red_pose.header.stamp = ros::Time::now();
    red_pose.header.frame_id = "world";
    red_pose.pose.position.x = turtle_config.x;
    red_pose.pose.position.y = turtle_config.y;

    red_path.poses.push_back(red_pose);

    path_pub.publish(red_path);
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
    nh.param("x0", x_0, -0.6);
    nh.param("y0", y_0, 0.8);
    nh.param("theta0", theta_0, 1.57);
    nh.param("x_length", x_length, 10.0);
    nh.param("y_length", y_length, 10.0);
    n.param("min_lidar_range", min_lidar_range,0.120);
    n.param("max_lidar_range", max_lidar_range,3.5);
    n.param("num_lidar_samples", num_lidar_samples,360);
    n.param("min_lidar_angle", min_lidar_angle,0.0);
    n.param("max_lidar_angle", max_lidar_angle,6.28319);
    n.param("lidar_angle_resolution", lidar_angle_resolution,1.0);
    n.param("lidar_range_resolution", lidar_range_resolution, 0.015);
    n.param("lidar_noise_mean", lidar_noise_mean, 0.0);
    n.param("lidar_noise_stddev", lidar_noise_stddev, 0.01);
    n.param("collision_radius", collision_radius, 0.11);

    nh.getParam("radii", radii);
    nh.getParam("x_pos", x_locs);
    nh.getParam("y_pos", y_locs);
    n.getParam("motor_cmd_to_radsec", motor_cmd_to_radsec);
    n.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);
    n.getParam("motor_cmd_max", motor_cmd_max);
    n.getParam("slip_min", slip_min);
    n.getParam("slip_max", slip_max);
    n.getParam("basic_sensor_variance", basic_sensor_variance);
    n.getParam("max_range", max_range);


    /// Setting up the looping rate and the required subscribers/publishers.
    ros::Rate r(frequency); 
    ros::Publisher ts_pub = nh.advertise<std_msgs::UInt64>("timestep", frequency);
    ros::Publisher obs_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles",10, true); //True means latched publisher
    ros::Publisher wall_pub = nh.advertise<visualization_msgs::MarkerArray>("walls",10, true); //True means latched publisher
    ros::Publisher sensor_data_pub = n.advertise<nuturtlebot_msgs::SensorData>("sensor_data",100);
    fake_sensor_pub = nh.advertise<visualization_msgs::MarkerArray>("fake_sensor",10);
    laser_scan_pub = n.advertise<sensor_msgs::LaserScan>("laser_data",10);

    ros::Subscriber wheel_cmd_sub = n.subscribe("wheel_cmd",100,wheelCallback); 

    /// Setting up the services, and the robot's initial location.
    ros::ServiceServer resetService = nh.advertiseService("reset", resetCallback);
    ros::ServiceServer advertiseService = nh.advertiseService("teleport", teleportCallback);

    ros::Timer timer = n.createTimer(ros::Duration(0.2), timerCallback);
    path_pub = n.advertise<nav_msgs::Path>("red_path", 100);

    ts.data = 0;
    wheel_angles.L = 0.0;
    wheel_angles.R = 0.0;
    wheel_angles_old = wheel_angles;
    turtle_config.theta = theta_0;
    turtle_config.x = x_0;
    turtle_config.y = y_0;
    drive.setConfig(turtle_config);

    /// Populating the MarkerArray messages and publishing it to display the markers.
    ma = addObstacles(radii, x_locs, y_locs);
    walls = addWalls(x_length, y_length);
    obs_pub.publish(ma);
    wall_pub.publish(walls);
    static tf2_ros::TransformBroadcaster br;

    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(slip_min,slip_max);


    /// The main loop of the node. Per the rate, this runs at 500Hz.
    while(ros::ok())
    {
        // ROS_ERROR_STREAM("making broadcaster");
        turtle_config = drive.getConfig();
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red-base_footprint";
        transformStamped.transform.translation.x = turtle_config.x;
        transformStamped.transform.translation.y = turtle_config.y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, turtle_config.theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        /// Send the transform and publish the JointState and the timestamp message.
        br.sendTransform(transformStamped);
        // joint_state_pub.publish(jointState);
        ts_pub.publish(ts);

        rand_slip = distribution(generator);
        // ROS_ERROR_STREAM(rand_slip);


        // Update the wheel positions and publish them on red/sensor_data as a nuturtlebot/SensorData message.
        // Convert ticks to radians/second, use the frequency to determine how far the wheels turn during that time
        sensor_data.left_encoder = toEncoderTicks(wheel_speeds.Ldot*rand_slip + (wheel_speeds.Ldot/frequency) + wheel_angles_old.L);
        sensor_data.right_encoder = toEncoderTicks(wheel_speeds.Rdot*rand_slip + (wheel_speeds.Rdot/frequency) + wheel_angles_old.R);
        sensor_data_pub.publish(sensor_data);

        wheel_angles.L = (wheel_speeds.Ldot/frequency) + wheel_angles_old.L;
        wheel_angles.R = (wheel_speeds.Rdot/frequency) + wheel_angles_old.R;

        // Save previous coordinates
        // Update coords
        // Check to see if you're near an obstacle
        // If you collided, set the pose such that the circles are tangent
        previous_config = drive.getConfig();
        turtle_config = drive.forward_kinematics(wheel_angles);

        for (int m=0; m<(int)ma.markers.size(); m++){

            // ROS_WARN_STREAM("turtle at :" << turtle_config.x << " " << turtle_config.y << "\r\n");
            // ROS_WARN_STREAM("marker at :" << ma.markers[m].pose.position.x << " " << ma.markers[m].pose.position.y << "\r\n");
            // ROS_WARN_STREAM(findDistance(0, 0, ma.markers[m].pose.position.x, ma.markers[m].pose.position.y) << "\r\n");
            
            if (findDistance(0, 0, ma.markers[m].pose.position.x, ma.markers[m].pose.position.y) < (collision_radius + (ma.markers[m].scale.x/2))){
                
                double theta = std::atan2(ma.markers[m].pose.position.y, ma.markers[m].pose.position.x);
                
                // turtlelib::Vector2D markervec = {x_locs[i], y_locs[i]};
                // turtlelib::Transform2D Twm(markervec);
                // turtlelib::Vector2D marker_in_world;

                turtlelib::Q new_config;
                new_config.x = x_locs[m] + (collision_radius + (ma.markers[m].scale.x/2)) * std::cos(theta);
                new_config.y = y_locs[m] + (collision_radius + (ma.markers[m].scale.y/2)) * std::sin(theta);
                new_config.theta = previous_config.theta;

                drive.setConfig(new_config);
            }
        }

        // wheel_angles_old.L = wheel_angles.L + wheel_speeds.Ldot*rand_slip;
        // wheel_angles_old.R = wheel_angles.R + wheel_speeds.Rdot*rand_slip;
        wheel_angles_old.L = wheel_angles.L;
        wheel_angles_old.R = wheel_angles.R;

        // Use forward kinematics from DiffDrive to update the position of the robot
        // if (ts.data % 100 == 0){
        //     // ROS_ERROR_STREAM(ts.data);
        //     ma = simulateObstacles(radii, x_locs, y_locs);
        //     fake_sensor_pub.publish(ma);

        //     simulateLidar();
        //     laser_scan_pub.publish(laserScan);
        //     // Publish LIDAR data
        // }


        /// Increment the timestamp, spin, and sleep for the 500Hz delay.
        ts.data++;
        ros::spinOnce();
        r.sleep();
    }
    return 0;   
}