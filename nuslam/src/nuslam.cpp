#include <iosfwd>
#include <iostream>
#include <vector>
#include <cmath>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "nuslam/nuslam.hpp"
#include <armadillo>
#include <ros/ros.h>


namespace nuslam{

    EKFilter::EKFilter(int n) :
            n(n),
            Xi(arma::mat(3+2*n,1)),
            K(arma::mat(3+2*n,3+2*n)),
            H(arma::mat(2,3+2*n)),
            Sigma(arma::mat(3+2*n,3+2*n)), //get rid of this
            Q(arma::mat(3+2*n,3+2*n, arma::fill::zeros)),
            R(arma::mat (2,2, arma::fill::eye)),
            q(arma::mat(3,1)),
            z_hat(arma::mat(2,1))

            {}

    // EKFilter::EKFilter(int n) :
    //         n(n),
    //         Xi(arma::mat(3+2*n,1)),
    //         K(arma::mat(3+2*n,3+2*n)),
    //         H(arma::mat(2,3+2*n)),
    //         Sigma(arma::mat(3+2*n,3+2*n)), //get rid of this
    //         Q(arma::mat(3+2*n,3+2*n)),
    //         R(arma::mat (2,2, arma::fill::eye)),
    //         q(arma::mat(3,1)),
    //         z_hat(arma::mat(2,1))

    //         {}

    void EKFilter::EKFilter_init(turtlelib::Q robot_state){
        // Initialize Xi, m, and Sigma to zero

        // Make the Xi state vector
        // Xi.set_size(3);
        Xi(0,0) = robot_state.theta;
        Xi(1,0) = robot_state.x;
        Xi(2,0) = robot_state.y;

        for(int i=0; i<n*2; i++){
            Sigma(i+3,i+3) = 100000;
        }

    }

    void EKFilter::UpdateMeasurement(int j){
        // Given a range and a bearing, return an H matrix
        // H matrix is of dimensions

        double deltaX = Xi((2*j)+3,0) - Xi(1,0);
        double deltaY = Xi((2*j)+4,0) - Xi(2,0);
        double d = std::pow(deltaX,2) + std::pow(deltaY,2);

        double rad = std::sqrt( std::pow(deltaX, 2) + std::pow(deltaY, 2) ); //this is r_j
        double angle = std::atan2( deltaY, deltaX) - Xi(0,0); // This is phi_j
        angle = turtlelib::normalize_angle(angle);
        
        // ROS_ERROR("Xi?");
        // Xi.print("");
        // ROS_ERROR("M?");
        // m.print();
        // ROS_ERROR_STREAM(m(2*j,0));
        // ROS_ERROR_STREAM(m(2*j+1,0));
        // ROS_ERROR_STREAM(Xi(0,0));
        // ROS_ERROR_STREAM(Xi(1,0));
        // ROS_ERROR_STREAM(rad);
        // ROS_ERROR_STREAM(angle);

        z_hat(0,0) = rad;
        z_hat(1,0) = angle;

        // ROS_ERROR("ZHAT");
        // z_hat.print();

        // n = m.n_rows;

        H = arma::mat(2,3+2*n, arma::fill::zeros);

        H(0,0) = 0;
        H(0,1) = -deltaX / std::sqrt(d);
        H(0,2) = -deltaY / std::sqrt(d);
        H(1,0) = -1;
        H(1,1) = deltaY / d;
        H(1,2) = -deltaX / d;

        H(0,2*j+3) = deltaX / std::sqrt(d);
        H(0,2*j+4) = deltaY / std::sqrt(d);
        H(1,2*j+3) = -deltaY / d;
        H(1,2*j+4) = deltaX / d;

    }

    void EKFilter::UpdateCovariance(){
        // Given:
        // Find A_t matrix, Using private Qbar
        // Make Sigma

        arma::mat KH = K*H;
        // ROS_WARN("KH matrix");
        // KH.print();
        arma::mat I = arma::eye(arma::size(KH));
        Sigma = (I - KH) * Sigma;
    }

    void EKFilter::UpdatePosState(double x, double y){
        // Make Xi using X and Y of the fake sensor data in the **map** frame
        arma::mat z(2,1);
        arma::mat z_hat_new(2,1);

        // Make Zhat

        // Make Z

        double rad = std::sqrt( std::pow(x, 2) + std::pow(y, 2) ); //this is z_j
        double angle = std::atan2( y, x ); // This is phi_j
        angle = turtlelib::normalize_angle(angle);
        
        z(0,0) = rad;
        z(1,0) = angle;

        arma::mat diff(2,1);
        diff = z-z_hat;
        diff(1,0) = turtlelib::normalize_angle(diff(1,0));

        Xi = Xi + (K * diff);

        Xi(0,0) = turtlelib::normalize_angle(Xi(0,0));

    }

    void EKFilter::ComputeKalmanGains(){
        // Make K

        // ROS_INFO_STREAM_ONCE("Sigma right before K" << Sigma);
        // Sigma.print();
        // H.print("H when calculating K");
        // Sigma.print("Sigma when calculating K");

        K = (Sigma * H.t()) * arma::inv((H * Sigma * H.t()) + R);

        // ROS_INFO_STREAM_ONCE("K first" << K);


    }

    void EKFilter::Predict(turtlelib::Twist2D twist){
        // Integrate twist, updating the estimate
        //Find A Matrix, Compute Sigma = At * Sigma * At.t() + Q_bar;
        arma::mat At_eye = arma::eye(3+2*n, 3+2*n);
        arma::mat At = arma::mat(3+2*n, 3+2*n, arma::fill::zeros);
        arma::mat A = arma::mat(3+2*n, 3+2*n, arma::fill::zeros);
        arma::mat Xi_update = arma::mat(3 + 2*n, 1, arma::fill::zeros);


        if(turtlelib::almost_equal(twist.thetadot, 0.0)){
            // If almost zero rotation:
            At(1,0) = -1*(twist.xdot) * std::sin(Xi(0,0));
            At(2,0) = (twist.xdot) * std::cos(Xi(0,0));

        }

        else{
            // Xi(0,0) += twist.thetadot;
            At(1,0) = ((-twist.xdot / twist.thetadot) * std::cos(Xi(0,0))) + ((twist.xdot / twist.thetadot) * std::cos( turtlelib::normalize_angle(Xi(0,0) + (twist.thetadot)) ));
            At(2,0) = ((-twist.xdot / twist.thetadot) * std::sin(Xi(0,0))) + ((twist.xdot / twist.thetadot) * std::sin( turtlelib::normalize_angle(Xi(0,0) + (twist.thetadot)) ));
        
        }

        A = At_eye + At;
        // A.print("At Matrix");

        Sigma = (A * Sigma * A.t()) + Q;
    }

    void EKFilter::init_landmarks(int marker_id, double x, double y){
        // Set the coordinates IN THE MAP FRAME
        // ROS_WARN_STREAM("init landmarks?");

        // double rad = std::sqrt( std::pow(x, 2) + std::pow(y, 2) ); //this is r_j
        // double angle = std::atan2( y, x ); // This is phi_j
        // angle = turtlelib::normalize_angle(angle);

        Xi((2*marker_id) + 3,0) = x;
        Xi((2*marker_id) + 4,0) = y;
    }

    void EKFilter::init_Q_R(double val, double rval){
        // ROS_WARN_STREAM("INIT QR");
        
        Q(0,0) = val;
        Q(1,1) = val;
        Q(2,2) = val;

        R *= rval;
        // R.print("R");
    }

    arma::mat EKFilter::get_Q(){
        return Q;
    }
    arma::mat EKFilter::get_H(){
        return H;
    }
    arma::mat EKFilter::get_Sigma(){
        return Sigma;
    }
    arma::mat EKFilter::get_K(){
        return K;
    }

    arma::mat EKFilter::get_R(){
        return R;
    }
    arma::colvec EKFilter::get_Xi(){
        return Xi;
    }
    arma::colvec EKFilter::get_q(){
        return q;
    }
    arma::colvec EKFilter::get_zhat(){
        return z_hat;
    }
    int EKFilter::get_n(){
        return n;
    }

}