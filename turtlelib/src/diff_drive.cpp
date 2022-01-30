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
    
    
    Q DiffDrive::forward_kinematics(Q current_config, Twist2D twist) {
        // Given a twist and an old Q, make a new Q
        Transform2D Tbb_prime(0);
        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = current_config.x;
        prev_vector.y = current_config.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        Q new_config;
        new_config.x = current_config.x + tbbp_vector.x;
        new_config.y = current_config.y + tbbp_vector.y;
        new_config.theta = normalize_angle(current_config.theta + twist.thetadot);
        
        return new_config;
    }

    Q DiffDrive::forward_kinematics(Q current_config, Phi prev_angle, Phi next_angle) {
        // Given a twist and an old Q, make a new Q

        Phidot wheel_speeds;
        wheel_speeds.Ldot = next_angle.L - prev_angle.L;
        wheel_speeds.Rdot = next_angle.R - prev_angle.R;

        Transform2D Tbb_prime(0);
        Twist2D twist;

        twist.thetadot = (0.5*r/d)*(wheel_speeds.Rdot - wheel_speeds.Ldot);
        twist.xdot = (r/2)*(wheel_speeds.Rdot + wheel_speeds.Ldot);
        twist.ydot = 0;       
        

        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = current_config.x;
        prev_vector.y = current_config.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        Q new_config;
        new_config.x = current_config.x + tbbp_vector.x;
        new_config.y = current_config.y + tbbp_vector.y;
        new_config.theta = normalize_angle(current_config.theta + twist.thetadot);
        
        return new_config;
    }

    Q DiffDrive::forward_kinematics(Q current_config, Phidot wheel_speeds) {

        Transform2D Tbb_prime(0);
        Twist2D twist;

        twist.thetadot = (0.5*r/d)*(wheel_speeds.Rdot - wheel_speeds.Ldot);
        twist.xdot = (r/2)*(wheel_speeds.Rdot + wheel_speeds.Ldot);
        twist.ydot = 0;       
        

        Tbb_prime = integrate_twist(twist);
        Vector2D prev_vector;
        prev_vector.x = current_config.x;
        prev_vector.y = current_config.y;

        Vector2D tbbp_vector = Tbb_prime(prev_vector);

        Q new_config;
        new_config.x = current_config.x + tbbp_vector.x;
        new_config.y = current_config.y + tbbp_vector.y;
        new_config.theta = normalize_angle(current_config.theta + twist.thetadot);
        
        return new_config;
    }

    Phidot DiffDrive::inverse_kinematics(Twist2D twist) {
        Phidot output;
        output.Ldot = ((-d)*twist.thetadot + twist.xdot)/r;
        output.Rdot = ((d)*twist.thetadot + twist.xdot)/r;

        if (twist.ydot != 0.0){
            throw std::logic_error("Invalid Twist!");
        }

        return output; //this goes to cmd_vel I guess?
    }

    Phi DiffDrive::update_phis(Twist2D twist, Phi angles){
            // call inverse_kinematics
        Phidot rates;
        rates = inverse_kinematics(twist);
        angles.L += rates.Ldot;
        angles.R += rates.Rdot;
        
        Phi output;
        output.L = std::fmod(angles.L, 2*PI);
        output.R = std::fmod(angles.R, 2*PI);
    
        return output;
    }

}