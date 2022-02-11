#include <iostream>
#include <string>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
/// \file
/// \brief The corresponding cpp file for "diff_drive.hpp", for differential drive robot kinematics.


namespace turtlelib{

    /// \brief initializes an empty diffdrive object
    DiffDrive::DiffDrive(){
        phi.L = 0;
        phi.R = 0;
        q.theta = 0;
        q.x = 0;
        q.y = 0;
        phidot.Ldot = 0;
        phidot.Rdot = 0;
    }
    
    /// \brief calculate the forward kinematics of the robot
    /// \param twist - the instanteous twist of the robot
    /// \return the updated turtlebot configuration
    Q DiffDrive::forward_kinematics(Twist2D twist) {

        Transform2D Tbb_prime, Twb_prime;

        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = q.x;
        prev_vector.y = q.y;

        Transform2D Twb(prev_vector, q.theta);

        Twb_prime = Twb * Tbb_prime;

        q.x = Twb_prime.translation().x;
        q.y = Twb_prime.translation().y;
        q.theta = normalize_angle(Twb_prime.rotation());
        
        return q;
    }
    
    /// \brief calculate the forward kinematics of the robot
    /// \param twist - the instanteous twist of the robot
    /// \param current_config - the current configuration of the robot (if not already in the object)
    /// \return the updated turtlebot configuration    
    Q DiffDrive::forward_kinematics(Q current_config, Twist2D twist) {
        // Given a twist and an old Q, make a new Q
        // Maybe overload with just a twist?
        Transform2D Tbb_prime(0);
        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = current_config.x;
        prev_vector.y = current_config.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        q.x = tbbp_vector.x;
        q.y = tbbp_vector.y;
        q.theta = normalize_angle(current_config.theta + twist.thetadot);
        
        return q;
    }


    /// \brief produces a twist from given wheel speeds
    /// \param speeds - the wheel speeds
    /// \return a twist from the wheel speeds
    Twist2D DiffDrive::get_twist_from_angles(Phidot speeds){
        // returns the instaneous twist

        Twist2D twist;

        twist.thetadot = (0.5*r/d)*(speeds.Rdot - speeds.Ldot);
        twist.xdot = (r/2)*(speeds.Rdot + speeds.Ldot);
        twist.ydot = 0;

        return twist;
    }

    /// \brief calculate the forward kinematics of the robot
    /// \param next_angle - the next wheel angles of the turtlebot after a timestep. Uses internal private variables to perform the rest of the calculations/
    /// returns the updated turtlebot configuration
    Q DiffDrive::forward_kinematics(Phi next_angle) {
        // Given a twist and an old Q, make a new Q
        // Updates wheel angles and wheel speeds as well


        // THIS IS THE ONE THAT WORKS

        phidot.Ldot = next_angle.L - phi.L;
        phidot.Rdot = next_angle.R - phi.R;

        Transform2D Tbb_prime, Twb_prime;
        Twist2D twist;

        twist.thetadot = (0.5*r/d)*(phidot.Rdot - phidot.Ldot);
        twist.xdot = (r/2)*(phidot.Rdot + phidot.Ldot);
        twist.ydot = 0;       
        

        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = q.x;
        prev_vector.y = q.y;

        Transform2D Twb(prev_vector, q.theta);

        Twb_prime = Twb * Tbb_prime;

        q.x = Twb_prime.translation().x;
        q.y = Twb_prime.translation().y;
        q.theta = normalize_angle(Twb_prime.rotation());

        phi.L = next_angle.L;
        phi.R = next_angle.R;
        
        return q;
    }

    /// \brief calculate the forward kinematics of the robot
    /// \param current_config - the current configuration of the robot (if not already in the object)
    /// \param prev_angle - the previous (or current) wheel angles
    /// \param next_angle - the next wheel angles of the turtlebot after a timestep
    /// \return the updated turtlebot configuration
    Q DiffDrive::forward_kinematics(Q current_config, Phi prev_angle, Phi next_angle) {
        // Given a twist and an old Q, make a new Q

        phidot.Ldot = next_angle.L - prev_angle.L;
        phidot.Rdot = next_angle.R - prev_angle.R;

        Transform2D Tbb_prime, Twb_prime;

        Twist2D twist;
        twist.thetadot = (0.5*r/d)*(phidot.Rdot - phidot.Ldot);
        twist.xdot = (r/2)*(phidot.Rdot + phidot.Ldot);
        twist.ydot = 0.0;
        
        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        double last_config_angle;

        prev_vector.x = current_config.x;
        prev_vector.y = current_config.y;
        last_config_angle = current_config.theta;

        Transform2D Twb(prev_vector, last_config_angle);

        Twb_prime = Twb*Tbb_prime;

        Vector2D new_config_vector = Twb_prime.translation();
        double new_angle = Twb_prime.rotation();

        q.x = new_config_vector.x;
        q.y = new_config_vector.y;
        q.theta = normalize_angle(new_angle);
        
        return q;
    }


    /// \brief calculate the forward kinematics of the robot
    /// \param wheel_speeds - the instanteous wheel speeds of the robot
    /// \param current_config - the current configuration of the robot (if not already in the object)
    /// \return the updated turtlebot configuration
    Q DiffDrive::forward_kinematics(Q current_config, Phidot wheel_speeds) {

        Transform2D Tbb_prime(0);
        Twist2D twist;
        
        phidot.Ldot = wheel_speeds.Ldot;
        phidot.Rdot = wheel_speeds.Rdot;

        twist.thetadot = (0.5*r/d)*(wheel_speeds.Rdot - wheel_speeds.Ldot);
        twist.xdot = (r/2)*(wheel_speeds.Rdot + wheel_speeds.Ldot);
        twist.ydot = 0;       
        

        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = current_config.x;
        prev_vector.y = current_config.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        // q.x = current_config.x + tbbp_vector.x;
        // q.y = current_config.y + tbbp_vector.y;
        q.x = tbbp_vector.x;
        q.y = tbbp_vector.y;
        q.theta = normalize_angle(current_config.theta + twist.thetadot);
        
        return q;
    }


    /// \brief calculate the forward kinematics of the robot
    /// \param twist - the instanteous twist of the robot
    /// \return the updated turtlebot configuration
    Phidot DiffDrive::inverse_kinematics(Twist2D twist) {
        phidot.Ldot = ((-d)*twist.thetadot + twist.xdot)/r;
        phidot.Rdot = ((d)*twist.thetadot + twist.xdot)/r;

        if (twist.ydot != 0.0){
            throw std::logic_error("Invalid Twist!");
        }

        return phidot; //this goes to cmd_vel I guess?
    }


    /// \brief updates the wheel angles given current angles and a twist
    /// \param twist - the instanteous body twist of the robot
    /// \param angles - the current wheel angles of the robot
    /// \return the updated angles
    Phi DiffDrive::update_phis(Twist2D twist, Phi angles){
            // call inverse_kinematics
        Phidot rates;
        phidot = inverse_kinematics(twist);
        angles.L += rates.Ldot;
        angles.R += rates.Rdot;
        
        phi.L = std::fmod(angles.L, 2*PI);
        phi.R = std::fmod(angles.R, 2*PI);
    
        return phi;
    }

    /// \brief sets the configuration of the robot
    /// \param config - the configuration of the robot
    void DiffDrive::setConfig(Q config){
        q = config;
    }

    /// \brief sets the wheel angles of the robot
    /// \param angles - the wheel angles of the robot
    void DiffDrive::setAngles(Phi angles){
        phi = angles;
    }

    /// \brief sets the wheel speeds of the robot
    /// \param speeds - the wheel speeds of the robot
    void DiffDrive::setSpeeds(Phidot speeds){
        phidot = speeds;
    }

    /// \brief returns the current robot configuration
    /// \return the current robot configuration
    Q DiffDrive::getConfig(){
        return q;
    }

    /// \brief returns the current wheel angles
    /// \return the current wheel angles
    Phi DiffDrive::getAngles(){
        return phi;
    }

    /// \brief returns the current wheel speeds
    /// \return the current wheel speeds
    Phidot DiffDrive::getSpeeds(){
        return phidot;
    }

}