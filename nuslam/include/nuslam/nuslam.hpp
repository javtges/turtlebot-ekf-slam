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


    class EKFilter
    {

    public:

    EKFilter();

    void EKFilter_init(turtlelib::Q robot_state, int n);

    void ComputeKalmanGains(); //Makes K

    void UpdatePosState(double x, double y); // Makes Xi

    void UpdateCovariance(); // Makes Sigma

    void UpdateMeasurement(int j); // Makes H

    void Predict(turtlelib::Twist2D twist, double time);

    void init_landmarks(int marker_id, double x, double y);

    void init_Q(double val);

    arma::mat get_Q();

    arma::mat get_H();

    arma::mat get_Sigma();

    arma::mat get_K();

    arma::colvec get_Xi();

    arma::colvec get_q();

    arma::colvec get_m();

    arma::colvec get_zhat();

    int get_n();

    private:
        arma::mat Q(2,2); // Process noise
        arma::mat H(2,9); // 
        arma::mat Sigma(9,9); // covariance matrix
        arma::mat K(9,9); // Kalman gain
        arma::colvec Xi(9); // Current state
        arma::colvec q(3); // Turtlebot pose 3x1
        arma::colvec m(6); // Marker locations 2nx1
        arma::colvec z_hat(2);
        int n = 3; //num rows
    };

}

#endif