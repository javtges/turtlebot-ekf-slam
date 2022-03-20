#include <catch_ros/catch.hpp>
#include <turtlelib/rigid2d.hpp>
#include <armadillo>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <ros/ros.h>
#include <ros/console.h>

static const double epsilon = 1.0e-4;


TEST_CASE("Circle Fitting Algorithm Case 1","Test Case 1"){ // James Avtges
    std::vector<turtlelib::Vector2D> clusters;

    clusters.resize(6);
    clusters.at(0) = {1,7};
    clusters.at(1) = {2,6};
    clusters.at(2) = {5,8};
    clusters.at(3) = {7,7};
    clusters.at(4) = {9,5};
    clusters.at(5) = {3,7};

    int n = 6;

    double x_bar = 0.0, y_bar = 0.0, x_sum = 0.0, y_sum = 0.0, z_sum = 0.0, z_bar = 0.0;
    double center_x, center_y, R;
    arma::mat Z(n, 4);

    for (int k=0; k<n; k++){ // loop through points in cluster
        x_sum += clusters.at(k).x;
        y_sum += clusters.at(k).y;
        // z_sum += std::pow(clusters.at(k).x, 2) + std::pow(clusters.at(k).y, 2);
    }

    // Find averages
    x_bar = x_sum / n ;
    y_bar = y_sum / n ;

    // z_bar = z_sum / n ;

    for (int l=0; l<n; l++){ // loop through points in cluster again
        double x_i = clusters.at(l).x - x_bar;
        double y_i = clusters.at(l).y - y_bar;

        z_sum += std::pow(x_i, 2) + std::pow(y_i, 2);

        Z(l,0) = std::pow(x_i, 2) + std::pow(y_i, 2);
        Z(l,1) = x_i;
        Z(l,2) = y_i;
        Z(l,3) = 1;
    }

    z_bar = z_sum / n;

    arma::mat M = (1/n) * Z.t() * Z;
    arma::mat H = arma::eye(4,4);
    H(0,0) = 8*z_bar;
    H(0,3) = 2;
    H(3,0) = 2;
    H(3,3) = 0;
    arma::mat H_inv = arma::eye(4,4);
    H_inv(0,0) = 0;
    H_inv(0,3) = 0.5;
    H_inv(3,0) = 0.5;
    H_inv(3,3) = -2*z_bar;

    // H_inv.print("H_inv");
    // H.print("H");
    // Z.print("Z");

    // SINGULAR VALUE DECOMPOSITION -----------------------

    arma::mat U;
    arma::colvec sig_vec;
    arma::mat V;
    arma::colvec A(4);
    arma::colvec A_star(4);

    arma::svd(U, sig_vec, V, Z);

    // U.print("U");
    // sig_vec.print("sigma");
    // V.print("V");
    arma::mat Sigma = arma::diagmat(sig_vec);

    double min_sigma = sig_vec.min();

    if (min_sigma < 1.0e-12){
        A = V.col(3);
    }
    else{
        arma::mat Y = V * Sigma * V.t();
        arma::mat Q = Y * arma::inv(H) * Y;

        arma::mat eigenvectors;
        arma::colvec eigenvalues;

        arma::eig_sym(eigenvalues, eigenvectors, Q);

        eigenvalues.print("eigenvalues");
        eigenvectors.print("eigenvectors");

        // double smallest_positive = 0.0;

        for(int val=0; val<=3; val++){
            if(eigenvalues(val) > 0 ){
                // smallest_positive = eigenvalues(val);
                A_star = eigenvectors.col(val);
                // print(smallest_positive);
                break;
            }
        }

        A = arma::inv(Y) * A_star;

    } // end else (min_sigma > 10e-12)

    // ROS_WARN_STREAM(x_bar << " " << y_bar << " " << z_bar);
    // Find location of the center and the radius of the circle

    // A.print("A");
    center_x = (-A(1) / (2*A(0))) + x_bar;
    center_y = (-A(2) / (2*A(0))) + y_bar;
    R = std::sqrt( (std::pow(A(1),2) + std::pow(A(2),2) - (4 * A(0) * A(3)) ) / (4 * std::pow(A(0),2)) );

    CHECK(center_x == Approx(4.615482).margin(epsilon));
    CHECK(center_y == Approx(2.807354).margin(epsilon));
    CHECK(R == Approx(4.8275).margin(epsilon));
}

TEST_CASE("Circle Fitting Algorithm Case 2","Test Case 2"){ // James Avtges
    std::vector<turtlelib::Vector2D> clusters;

    clusters.resize(4);
    clusters.at(0) = {-1,0};
    clusters.at(1) = {-0.3,-0.06};
    clusters.at(2) = {0.3,0.1};
    clusters.at(3) = {1,0};

    int n = 4;

    double x_bar = 0.0, y_bar = 0.0, x_sum = 0.0, y_sum = 0.0, z_sum = 0.0, z_bar = 0.0;
    double center_x, center_y, R;
    arma::mat Z(n, 4);

    for (int k=0; k<n; k++){ // loop through points in cluster
        x_sum += clusters.at(k).x;
        y_sum += clusters.at(k).y;
        // z_sum += std::pow(clusters.at(k).x, 2) + std::pow(clusters.at(k).y, 2);
    }

    // Find averages
    x_bar = x_sum / n ;
    y_bar = y_sum / n ;

    // z_bar = z_sum / n ;

    for (int l=0; l<n; l++){ // loop through points in cluster again
        double x_i = clusters.at(l).x - x_bar;
        double y_i = clusters.at(l).y - y_bar;

        z_sum += std::pow(x_i, 2) + std::pow(y_i, 2);

        Z(l,0) = std::pow(x_i, 2) + std::pow(y_i, 2);
        Z(l,1) = x_i;
        Z(l,2) = y_i;
        Z(l,3) = 1;
    }

    z_bar = z_sum / n;

    arma::mat M = (1/n) * Z.t() * Z;
    arma::mat H = arma::eye(4,4);
    H(0,0) = 8*z_bar;
    H(0,3) = 2;
    H(3,0) = 2;
    H(3,3) = 0;
    arma::mat H_inv = arma::eye(4,4);
    H_inv(0,0) = 0;
    H_inv(0,3) = 0.5;
    H_inv(3,0) = 0.5;
    H_inv(3,3) = -2*z_bar;

    // H_inv.print("H_inv");
    // H.print("H");
    // Z.print("Z");

    // SINGULAR VALUE DECOMPOSITION -----------------------

    arma::mat U;
    arma::colvec sig_vec;
    arma::mat V;
    arma::colvec A(4);
    arma::colvec A_star(4);

    arma::svd(U, sig_vec, V, Z);

    // U.print("U");
    // sig_vec.print("sigma");
    // V.print("V");
    arma::mat Sigma = arma::diagmat(sig_vec);

    double min_sigma = sig_vec.min();

    if (min_sigma < 1.0e-12){
        A = V.col(3);
    }
    else{
        arma::mat Y = V * Sigma * V.t();
        arma::mat Q = Y * arma::inv(H) * Y;

        arma::mat eigenvectors;
        arma::colvec eigenvalues;

        arma::eig_sym(eigenvalues, eigenvectors, Q);

        eigenvalues.print("eigenvalues");
        eigenvectors.print("eigenvectors");

        // double smallest_positive = 0.0;

        for(int val=0; val<=3; val++){
            if(eigenvalues(val) > 0 ){
                // smallest_positive = eigenvalues(val);
                A_star = eigenvectors.col(val);
                // print(smallest_positive);
                break;
            }
        }

        A = arma::inv(Y) * A_star;

    } // end else (min_sigma > 10e-12)

    // ROS_WARN_STREAM(x_bar << " " << y_bar << " " << z_bar);
    // Find location of the center and the radius of the circle

    // A.print("A");
    center_x = (-A(1) / (2*A(0))) + x_bar;
    center_y = (-A(2) / (2*A(0))) + y_bar;
    R = std::sqrt( (std::pow(A(1),2) + std::pow(A(2),2) - (4 * A(0) * A(3)) ) / (4 * std::pow(A(0),2)) );

    CHECK(center_x == Approx(0.4908357).margin(epsilon));
    CHECK(center_y == Approx(-22.15212).margin(epsilon));
    CHECK(R == Approx(22.17979).margin(epsilon));
}