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
            Xi(arma::mat(9,1)),
            K(arma::mat(3+2*n,3+2*n)),
            H(arma::mat(2,3+2*n)),
            Sigma(arma::mat(3+2*n,3+2*n)),
            m(arma::mat(2*n,1)),
            Q(arma::mat(3+2*n,3+2*n)),
            q(arma::mat(3,1)),
            z_hat(arma::mat(2,1))

            {}

    void EKFilter::EKFilter_init(turtlelib::Q robot_state, int num){
        // Initialize Xi, m, and Sigma to zero

        // Make the Xi state vector
        // Xi.set_size(3);
        Xi(0,0) = robot_state.theta;
        Xi(1,0) = robot_state.x;
        Xi(2,0) = robot_state.y;
        // m.set_size(2*n);
        // m.fill(0.0);

        // Xi = join_cols(Xi, m);
        // Need to make the m part of the Xi

        // Initialize Sigma to all Zeros
        // arma::mat Sigma_0q(3, 3, arma::fill::zeros);
        // arma::mat zero_3x2n(3, 2*n, arma::fill::zeros);
        // arma::mat zero_2nx3(2*n, 3, arma::fill::zeros);
        // arma::vec sigma_diag(n, 1000000);
        // arma::mat Sigma_0m = arma::diagmat(sigma_diag);

        // arma::mat top = arma::join_rows(Sigma_0q, zero_3x2n);
        // arma::mat bot = arma::join_rows(zero_2nx3, Sigma_0m);

        for(int i=0; i<n*2; i++){
            Sigma(i+3,i+3) = 1000;
        }

        // Sigma(3,3) = 10000;
        // Sigma(4,4) = 10000;
        // Sigma(5,5) = 10000;

        // Sigma = arma::join_cols(top, bot);

    }

    void EKFilter::UpdateMeasurement(int j){
        // Given a range and a bearing, return an H matrix
        // H matrix is of dimensions

        double rad = std::sqrt( std::pow(Xi(2*j+3,0) - Xi(1,0), 2) + std::pow(Xi(2*j+4,0) - Xi(2,0), 2) ); //this is r_j
        double angle = std::atan2( Xi(2*j+4,0), Xi(2*j+3,0)) - Xi(0,0); // This is phi_j
        angle = turtlelib::normalize_angle(angle);
        
        
        ROS_ERROR("Xi?");
        Xi.print();
        ROS_ERROR("M?");
        m.print();
        ROS_ERROR_STREAM(m(2*j,0));
        ROS_ERROR_STREAM(m(2*j+1,0));
        ROS_ERROR_STREAM(Xi(0,0));
        ROS_ERROR_STREAM(Xi(1,0));
        ROS_ERROR_STREAM(rad);
        ROS_ERROR_STREAM(angle);

        z_hat(0,0) = rad;
        z_hat(1,0) = angle;

        ROS_ERROR("ZHAT");
        z_hat.print();

        double deltaX = Xi(2*j+3,0) - Xi(0,0);
        double deltaY = Xi(2*j + 4,0) - Xi(1,0);
        double d = std::pow(deltaX,2) + std::pow(deltaY,2);
        // n = m.n_rows;

        H = arma::mat(2,3+2*n, arma::fill::zeros);

        H(0,0) = 0;
        H(0,1) = -deltaX / std::sqrt(d);
        H(0,2) = deltaY / d;
        H(1,0) = -1;
        H(1,1) = -deltaY / std::sqrt(d);
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
        ROS_WARN("KH matrix");
        KH.print();
        arma::mat I = arma::eye(arma::size(KH));
        Sigma = (I - KH) * Sigma;
    }

    void EKFilter::UpdatePosState(double x, double y){
        // Make Xi using X and Y of the fake sensor data in the map frame
        arma::mat z(2,1);
        z(0,0) = std::sqrt( std::pow(x,2) + std::pow(y,2) );
        z(1,0) = std::atan2( y, x );
        z(1,0) = turtlelib::normalize_angle(z(1,0));

        arma::mat diff(2,1);
        diff = z-z_hat;
        diff(1,0) = turtlelib::normalize_angle(diff(1,0));

        ROS_ERROR("XI");
        // z.print();
        // z_hat.print();
        // K.print();
        // Xi.print();

        Xi = Xi + K * diff;

        // Xi.print();
    }

    void EKFilter::ComputeKalmanGains(){
        // Make K
        arma::mat R(2,2, arma::fill::eye);
        R *= 0.001;

        K = Sigma * H.t() * (H * Sigma * H.t() + R).i();

    }

    void EKFilter::Predict(turtlelib::Twist2D twist, double time){
        // Integrate twist, updating the estimate
        //Find A Matrix, Compute Sigma = At * Sigma * At.t() + Q_bar;
        int length = Xi.n_rows;
        arma::mat At = arma::eye(3+2*n, 3+2*n);

        if(turtlelib::almost_equal(twist.thetadot, 0.0)){
            // If almost zero rotation:
            Xi(0,0) += 0;
            Xi(1,0) += twist.xdot * std::cos(Xi(0)) / time; 
            Xi(2,0) += twist.xdot * std::sin(Xi(0)) / time;

            At(1,0) += (-twist.xdot/time) * std::sin(Xi(0));
            At(2,0) += (twist.xdot/time) * std::cos(Xi(0));
        }

        else{

            Xi(0,0) += twist.thetadot / time;
            Xi(0,0) = turtlelib::normalize_angle(Xi(0,0));
            Xi(1,0) += ((-twist.xdot / twist.thetadot) * std::sin(Xi(0))) + ((twist.xdot / twist.thetadot) * std::sin(Xi(0) + (twist.thetadot/time)));
            Xi(2,0) += ((twist.xdot / twist.thetadot) * std::cos(Xi(0))) - ((twist.xdot / twist.thetadot) * std::cos(Xi(0) + (twist.thetadot/time)));
        
            At(1,0) += ((-twist.xdot / twist.thetadot) * std::cos(Xi(0))) + ((twist.xdot / twist.thetadot) * std::cos(Xi(0) + (twist.thetadot/time)));
            At(2,0) += ((-twist.xdot / twist.thetadot) * std::sin(Xi(0))) + ((twist.xdot / twist.thetadot) * std::sin(Xi(0) + (twist.thetadot/time)));
        }

        At.print();

        Sigma = At * Sigma * At.t() + Q;
    }

    void EKFilter::init_landmarks(int marker_id, double x, double y){
        // Set the coordinates IN THE MAP FRAME
        m(2*marker_id, 0) = x;
        m((2*marker_id) + 1 , 0) = y;

        Xi((2*marker_id) + 3,0) = x;
        Xi((2*marker_id) + 4,0) = y;
    }

    void EKFilter::init_Q(double val){
        for(int i=0; i<3; i++){
            Q(i,i) = val;
        }
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
    arma::colvec EKFilter::get_Xi(){
        return Xi;
    }
    arma::colvec EKFilter::get_q(){
        return q;
    }
    arma::colvec EKFilter::get_m(){
        return m;
    }
    arma::colvec EKFilter::get_zhat(){
        return z_hat;
    }
    int EKFilter::get_n(){
        return n;
    }

}