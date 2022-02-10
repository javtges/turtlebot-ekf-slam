#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP


#include <iosfwd>
#include <iostream>
#include <vector>
#include <cmath>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{

    constexpr double d = 0.16/2;
    constexpr double r = 0.033;

    struct Phi{
        double L = 0.0;
        double R = 0.0;
    };

    struct Q {
        double theta = 0.0;
        double x = 0.0;
        double y = 0.0;
    };

    struct Phidot{
        double Ldot = 0.0;
        double Rdot = 0.0;
    };

    class DiffDrive
    {
    private:
        //private?
        // std::vector<double> q = {0.0, 0.0, 0.0};
        // std::vector<double> phi = {0.0, 0.0};
        Phi phi;
        Q q;
        Phidot phidot;

    public:
        DiffDrive();

        Q forward_kinematics(Twist2D twist);

        Q forward_kinematics(Q current_config, Twist2D twist);

        Q forward_kinematics(Q current_config, Phidot wheel_speeds);
        
        Q forward_kinematics(Q current_config, Phi prev_angle, Phi next_angle);

        Q forward_kinematics(Phi next_angle);

        // Q forward_kinematics(Phi next_angle);

        Phidot inverse_kinematics(Twist2D twist);

        Phi update_phis(Twist2D twist, Phi angles);

        Twist2D get_twist_from_angles(Phidot speeds);

        void setConfig(Q config);
        void setAngles(Phi angles);
        void setSpeeds(Phidot speeds);
        Q getConfig();
        Phi getAngles();
        Phidot getSpeeds();

        //public?

        //Compute q
        // Takes a twist and a wheel radius, finds q

        //Forward kinematics function
        // Given new wheel positions update the configuration q

        //Inverse kinematics function(twist)
        // Compute the wheel velocities required to achieve a certain body twist

    };

}

#endif