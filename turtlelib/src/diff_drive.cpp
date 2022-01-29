#include <iostream>
#include <string>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"


namespace turtlelib{

    Q DiffDrive::forward_kinematics(Q current_config, Twist2D twist) const{
        // Given a twist and an old Q, make a new Q

    }

    Phidot DiffDrive::inverse_kinematics(Twist2D twist) {
        Phidot output;
        output.Ldot = ((-0.08)*twist.thetadot + twist.xdot)/0.033;
        output.Rdot = ((0.08)*twist.thetadot + twist.xdot)/0.033;

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