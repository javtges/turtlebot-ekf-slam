#include <iostream>
#include <string>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"


namespace turtlelib{

    DiffDrive::DiffDrive(){
        phi.L = 0;
        phi.R = 0;
        q.theta = 0;
        q.x = 0;
        q.y = 0;
        phidot.Ldot = 0;
        phidot.Rdot = 0;
    }
    
    Q DiffDrive::forward_kinematics(Twist2D twist) {
        // Given a twist and an old Q, make a new Q
        // Maybe overload with just a twist?
        Transform2D Tbb_prime(0);
        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = q.x;
        prev_vector.y = q.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        q.x = q.x + tbbp_vector.x;
        q.y = q.y + tbbp_vector.y;
        q.theta = normalize_angle(q.theta + twist.thetadot);
        
        return q;
    }
    
    Q DiffDrive::forward_kinematics(Q current_config, Twist2D twist) {
        // Given a twist and an old Q, make a new Q
        // Maybe overload with just a twist?
        Transform2D Tbb_prime(0);
        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = current_config.x;
        prev_vector.y = current_config.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        q.x = current_config.x + tbbp_vector.x;
        q.y = current_config.y + tbbp_vector.y;
        q.theta = normalize_angle(current_config.theta + twist.thetadot);
        
        return q;
    }

    Twist2D DiffDrive::get_twist_from_angles(Phi prev_angle, Phi next_angle){
        
        phidot.Ldot = next_angle.L - prev_angle.L;
        phidot.Rdot = next_angle.R - prev_angle.R;

        Twist2D twist;

        twist.thetadot = (0.5*r/d)*(phidot.Rdot - phidot.Ldot);
        twist.xdot = (r/2)*(phidot.Rdot + phidot.Ldot);
        twist.ydot = 0;

        return twist;
    }

    Q DiffDrive::forward_kinematics(Phi prev_angle, Phi next_angle) {
        // Given a twist and an old Q, make a new Q

        phidot.Ldot = next_angle.L - prev_angle.L;
        phidot.Rdot = next_angle.R - prev_angle.R;

        Transform2D Tbb_prime(0);
        Twist2D twist;

        twist.thetadot = (0.5*r/d)*(phidot.Rdot - phidot.Ldot);
        twist.xdot = (r/2)*(phidot.Rdot + phidot.Ldot);
        twist.ydot = 0;       
        

        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = q.x;
        prev_vector.y = q.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        q.x = q.x + tbbp_vector.x;
        q.y = q.y + tbbp_vector.y;
        q.theta = normalize_angle(q.theta + twist.thetadot);
        
        return q;
    }

    Q DiffDrive::forward_kinematics(Q current_config, Phi prev_angle, Phi next_angle) {
        // Given a twist and an old Q, make a new Q

        phidot.Ldot = next_angle.L - prev_angle.L;
        phidot.Rdot = next_angle.R - prev_angle.R;

        Transform2D Tbb_prime(0);
        Twist2D twist;

        twist.thetadot = (0.5*r/d)*(phidot.Rdot - phidot.Ldot);
        twist.xdot = (r/2)*(phidot.Rdot + phidot.Ldot);
        twist.ydot = 0;       
        

        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = q.x;
        prev_vector.y = q.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        q.x = q.x + tbbp_vector.x;
        q.y = q.y + tbbp_vector.y;
        q.theta = normalize_angle(q.theta + twist.thetadot);
        
        return q;
    }

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

        q.x = current_config.x + tbbp_vector.x;
        q.y = current_config.y + tbbp_vector.y;
        q.theta = normalize_angle(current_config.theta + twist.thetadot);
        
        return q;
    }

    Phidot DiffDrive::inverse_kinematics(Twist2D twist) {
        phidot.Ldot = ((-d)*twist.thetadot + twist.xdot)/r;
        phidot.Rdot = ((d)*twist.thetadot + twist.xdot)/r;

        if (twist.ydot != 0.0){
            throw std::logic_error("Invalid Twist!");
        }

        return phidot; //this goes to cmd_vel I guess?
    }

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

    void DiffDrive::setConfig(Q config){
        q = config;
    }
    void DiffDrive::setAngles(Phi angles){
        phi = angles;
    }
    void DiffDrive::setSpeeds(Phidot speeds){
        phidot = speeds;
    }
    Q DiffDrive::getConfig(){
        return q;
    }
    Phi DiffDrive::getAngles(){
        return phi;
    }
    Phidot DiffDrive::getSpeeds(){
        return phidot;
    }

}