#ifndef DIFF_DRIVE_INCLUDE_GUARD_HPP
#define DIFF_DRIVE_INCLUDE_GUARD_HPP


#include <iosfwd>
#include <iostream>
#include <vector>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
/// \file
/// \brief Handling the kinematics of a differetial-drive robot

namespace turtlelib
{

    /// \brief the parameters of the turtlebot. 'd' is the base-to-wheel distance, and 'r' is the wheel radius.
    constexpr double d = 0.16/2;
    constexpr double r = 0.033;

    /// \brief the wheel angles
    struct Phi{
        double L = 0.0;
        double R = 0.0;
    };

    /// \brief the turtlebot configuration
    struct Q {
        double theta = 0.0;
        double x = 0.0;
        double y = 0.0;
    };

    /// \brief the wheel speeds
    struct Phidot{
        double Ldot = 0.0;
        double Rdot = 0.0;
    };

    class DiffDrive
    {
    private:
        Phi phi;
        Q q;
        Phidot phidot;

    public:
        /// \brief initialize the differential drive object.
        DiffDrive();

        /// \brief calculate the forward kinematics of the robot
        /// \param twist - the instanteous twist of the robot
        /// \return the updated turtlebot configuration
        Q forward_kinematics(Twist2D twist);

        /// \brief calculate the forward kinematics of the robot
        /// \param twist - the instanteous twist of the robot
        /// \param current_config - the current configuration of the robot (if not already in the object)
        /// \return the updated turtlebot configuration
        Q forward_kinematics(Q current_config, Twist2D twist);

        /// \brief calculate the forward kinematics of the robot
        /// \param wheel_speeds - the instanteous wheel speeds of the robot
        /// \param current_config - the current configuration of the robot (if not already in the object)
        /// \return the updated turtlebot configuration
        Q forward_kinematics(Q current_config, Phidot wheel_speeds);
        
        /// \brief calculate the forward kinematics of the robot
        /// \param current_config - the current configuration of the robot (if not already in the object)
        /// \param prev_angle - the previous (or current) wheel angles
        /// \param next_angle - the next wheel angles of the turtlebot after a timestep
        /// \return the updated turtlebot configuration
        Q forward_kinematics(Q current_config, Phi prev_angle, Phi next_angle);

        /// \brief calculate the forward kinematics of the robot
        /// \param next_angle - the next wheel angles of the turtlebot after a timestep. Uses internal private variables to perform the rest of the calculations/
        /// returns the updated turtlebot configuration
        Q forward_kinematics(Phi next_angle);

        /// \brief calculates the wheel speeds given a body twist
        /// \param twist - the instanteous body twist of the robot
        /// \return the instanteous wheel speeds to produce the given twist. Throws an error if the twist is invalid.
        Phidot inverse_kinematics(Twist2D twist);

        /// \brief updates the wheel angles given current angles and a twist
        /// \param twist - the instanteous body twist of the robot
        /// \param angles - the current wheel angles of the robot
        /// \return the updated angles
        Phi update_phis(Twist2D twist, Phi angles);

        /// \brief produces a twist from given wheel speeds
        /// \param speeds - the wheel speeds
        /// \return a twist from the wheel speeds
        Twist2D get_twist_from_angles(Phidot speeds);

        /// \brief sets the configuration of the robot
        /// \param config - the configuration of the robot
        void setConfig(Q config);

        /// \brief sets the wheel angles of the robot
        /// \param angles - the wheel angles of the robot
        void setAngles(Phi angles);

        /// \brief sets the wheel speeds of the robot
        /// \param speeds - the wheel speeds of the robot
        void setSpeeds(Phidot speeds);

        /// \brief returns the current robot configuration
        /// \return the current robot configuration
        Q getConfig();

        /// \brief returns the current wheel angles
        /// \return the current wheel angles
        Phi getAngles();

        /// \brief returns the current wheel speeds
        /// \return the current wheel speeds
        Phidot getSpeeds();

    };

}

#endif