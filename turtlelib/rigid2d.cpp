#include <iostream>
#include <string>
#include "rigid2d.hpp"
#include <cmath>


namespace turtlelib{

    Transform2D::Transform2D(Vector2D trans){

        T[0][0] = 1;
        T[1][0] = 0;
        T[2][0] = 0;
        T[0][1] = 0;
        T[1][1] = 1;
        T[2][1] = 0;
        T[0][2] = trans.x;
        T[1][2] = trans.y;
        T[2][2] = 1;
    }

    Transform2D::Transform2D(double radians){

        T[0][0] = cos(deg2rad(radians));
        T[1][0] = sin(deg2rad(radians));
        T[2][0] = 0;
        T[0][1] = -sin(deg2rad(radians));
        T[1][1] = cos(deg2rad(radians));
        T[2][1] = 0;
        T[0][2] = 0;
        T[1][2] = 0;
        T[2][2] = 1;

    }


    Transform2D::Transform2D(Vector2D trans, double radians){

        T[0][0] = cos(deg2rad(radians));
        T[1][0] = sin(deg2rad(radians));
        T[2][0] = 0;
        T[0][1] = -sin(deg2rad(radians));
        T[1][1] = cos(deg2rad(radians));
        T[2][1] = 0;
        T[0][2] = trans.x;
        T[1][2] = trans.y;
        T[2][2] = 1;

    }

    Vector2D Transform2D::operator()(Vector2D v) const{

        // float v_arr[3][1];
        // v_arr[0][0] = v.x;
        // v_arr[0][0] = v.y;
        // v_arr[0][0] = 1;

        Vector2D outVec;
        outVec.x = T[0][0]*v.x + T[0][1]*v.y + T[0][2]*1;
        outVec.y = T[1][0]*v.x + T[1][1]*v.y + T[1][2]*1;

        return outVec;
    }


    Transform2D Transform2D::inv() const{
        Transform2D Tinv(0);

        Tinv.T[0][0] = T[0][0];
        Tinv.T[1][0] = T[0][1];
        Tinv.T[2][0] = 0;
        Tinv.T[0][1] = T[1][0];
        Tinv.T[1][1] = T[1][1];
        Tinv.T[2][1] = 0;
        Tinv.T[0][2] = -1*T[0][2];
        Tinv.T[1][2] = -1*T[1][2];
        Tinv.T[2][2] = 1;

        return Tinv;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){

        //Tab*=Tbc;
        //Tac = (Tab *= Tab);
        //Tac*=Tbc;

        T[0][0] = T[0][0]*rhs.T[0][0] + T[0][1]*rhs.T[1][0] + T[0][2]*rhs.T[2][0];
        T[1][0] = T[1][0]*rhs.T[0][0] + T[1][1]*rhs.T[1][0] + T[1][2]*rhs.T[2][0];
        T[2][0] = T[2][0]*rhs.T[0][0] + T[2][1]*rhs.T[1][0] + T[2][2]*rhs.T[2][0];
        T[0][1] = T[0][0]*rhs.T[0][1] + T[0][1]*rhs.T[1][1] + T[0][2]*rhs.T[2][1];
        T[1][1] = T[1][0]*rhs.T[0][1] + T[1][1]*rhs.T[1][1] + T[1][2]*rhs.T[2][1];
        T[2][1] = T[2][0]*rhs.T[0][1] + T[2][1]*rhs.T[1][1] + T[2][2]*rhs.T[2][1];
        T[0][2] = T[0][0]*rhs.T[0][2] + T[0][1]*rhs.T[1][2] + T[0][2]*rhs.T[2][2];
        T[1][2] = T[1][0]*rhs.T[0][2] + T[1][1]*rhs.T[1][2] + T[1][2]*rhs.T[2][2];
        T[2][2] = T[2][0]*rhs.T[0][2] + T[2][1]*rhs.T[1][2] + T[2][2]*rhs.T[2][2];

        return *this;
    }

    Vector2D Transform2D::translation() const{
        Vector2D newvec;
        newvec.x = T[0][0];
        newvec.y = T[0][1];
        return newvec;
    }

    double Transform2D::rotation() const{
        double newrot;
        newrot = acos(T[0][0]);
        return newrot;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }


    std::ostream & operator<<(std::ostream & os, const Twist2D & v){
        os << "[" << v.xdot << " " << v.ydot << " " << v.thetadot << "]\n";
        return os;
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]\n";
        return os;
    }

    std::istream & operator>>(std::istream & is, Transform2D & tf){
        Vector2D vec;
        double theta;
        char str[6];
        char c1;
        c1 = is.peek();
        if (c1 == 'd'){

            is.get(str,6);
            is >> theta;
            is.get(str,4);
            is >> vec.x;
            is.get(str,4);
            is >> vec.y;

            Transform2D output(vec,theta);
            tf = output;

        }

        return is;
    }

    std::istream & operator>>(std::istream & is, Twist2D & v){
        char c1;
        c1 = is.peek();
        if (c1 == '['){
            is.get();
        }
        
        is >> v.xdot >> v.ydot >> v.thetadot;

        return is;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        
        char c1;
        c1 = is.peek();
        if (c1 == '['){
            is.get();
        }
       
        is >> v.x >> v.y;

        return is;
    }



    // std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        // os << "deg: " << rad2deg(acos(tf.rotation())) << "x: " << tf.translation().x << "y: " << tf.translation().y;
        // return os;
    // }


}