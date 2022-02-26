#include <iosfwd>
#include <iostream>
#include <vector>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuslam/nuslam.hpp"
#include <armadillo>


namespace nuslam{


    ExtendedKalmanFilter::ExtendedKalmanFilter(turtlelib::Q robot_state, int n){
        // Initialize everything to zero

        // Make the Xi state vector
        Xi.set_size(3);
        Xi(0) = robot_state.x;
        Xi(1) = robot_state.y;
        Xi(2) = robot_state.theta;
        m.set_size(2*n);
        m.fill(0.0);

        Xi = join_cols(Xi, m);
        // Need to make the m part of the Xi


        // Initialize Sigma
        arma::mat Sigma_0q(3, 3, arma::fill::zeros);
        arma::mat zero_3x2n(3, 2*n, arma::fill::zeros);
        arma::mat zero_2nx3(2*n, 3, arma::fill::zeros);
        arma::vec sigma_diag(n, 1000000);
        arma::mat Sigma_0m = arma::diagmat(sigma_diag);

        arma::mat top = join_rows(Sigma_0q, zero_3x2n);
        arma::mat bot = join_rows(zero_2nx3, Sigma_0m);

        Sigma = join_cols(top, bot);

    }

    void ExtendedKalmanFilter::UpdateMeasurement(int j){
        // Given a range and a bearing, return an H matrix
        // H matrix is of dimensions 

    }


    void ExtendedKalmanFilter::UpdateCovariance(){
        // Given:
        // Find A_t matrix, Using private Qbar
        // Make Sigma
    }

    void ExtendedKalmanFilter::UpdatePosState(){
        // Make Xi
    }

    void ExtendedKalmanFilter::ComputeKalmanGains(){
        // Make K
    }







}