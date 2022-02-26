#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP

#include <iosfwd>
#include <iostream>
#include <vector>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>

namespace nuslam
{

    // Structs?


    class ExtendedKalmanFilter
    {

    public:

    ExtendedKalmanFilter(turtlelib::Q robot_state, int n);

    void ComputeKalmanGains(); //Makes K

    void UpdatePosState(); // Makes Xi

    void UpdateCovariance(); // Makes Sigma

    void UpdateMeasurement(int j); // Makes H

    // Make theoretical measurement


    private:
        arma::mat Q_bar; // Process noise
        arma::mat H; // 
        arma::mat Sigma; // covariance matrix
        arma::mat K; // Kalman gain
        arma::colvec Xi; // Current state
        arma::colvec q; // Turtlebot pose 3x1
        arma::colvec m; // Marker locations 2nx1
    };

}

#endif