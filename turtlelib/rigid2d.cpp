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

        Vector2D outVec;
        outVec.x = T[0][0]*v.x + T[0][1]*v.y + T[0][2]*1;
        outVec.y = T[1][0]*v.x + T[1][1]*v.y + T[1][2]*1;

        return outVec;
    }

    Vector2D Transform2D::normalize(Vector2D v) const{
        Vector2D output;
        
        double mag = sqrt(v.x * v.x + v.y * v.y);
        output.x = v.x/mag;
        output.y = v.y/mag;

        return output;
    }

    Twist2D Transform2D::operator()(Twist2D v) const{

        Twist2D outTwist;
        outTwist.thetadot = 0 * v.xdot + 0 * v.ydot + v.thetadot;
        outTwist.xdot = T[1][2]*v.thetadot + T[0][0]*v.xdot + T[0][1]*v.ydot;
        outTwist.ydot = -1*T[0][2] * v.thetadot + T[1][0] * v.xdot + T[1][1] * v.ydot;

        return outTwist;
    }

    Transform2D Transform2D::inv() const{
        Transform2D Tinv1(0);
        Transform2D Tinv2(0);

        Tinv1.T[0][0] = T[0][0];
        Tinv1.T[1][0] = T[0][1];
        Tinv1.T[2][0] = 0;
        Tinv1.T[0][1] = T[1][0];
        Tinv1.T[1][1] = T[1][1];
        Tinv1.T[2][1] = 0;
        Tinv1.T[0][2] = 0;
        Tinv1.T[1][2] = 0;
        Tinv1.T[2][2] = 1;

        Tinv2.T[0][0] = 1;
        Tinv2.T[1][0] = 0;
        Tinv2.T[2][0] = 0;
        Tinv2.T[0][1] = 0;
        Tinv2.T[1][1] = 1;
        Tinv2.T[2][1] = 0;
        Tinv2.T[0][2] = -1*T[0][2];
        Tinv2.T[1][2] = -1*T[1][2];
        Tinv2.T[2][2] = 1;

        return Tinv1 * Tinv2;
    }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){

        //Tab*=Tbc;
        //Tac = (Tab *= Tab);
        //Tac*=Tbc;

        // Thanks to Anna Garverick for the idea to use constant values for the LHS indices!

        const float t00 = T[0][0];
        const float t01 = T[0][1];
        const float t02 = T[0][2];
        const float t10 = T[1][0];
        const float t11 = T[1][1];
        const float t12 = T[1][2];
        const float t20 = T[2][0];
        const float t21 = T[2][1];
        const float t22 = T[2][2];

        T[0][0] = t00*rhs.T[0][0] + t01*rhs.T[1][0] + t02*rhs.T[2][0];
        T[1][0] = t10*rhs.T[0][0] + t11*rhs.T[1][0] + t12*rhs.T[2][0];
        T[2][0] = t20*rhs.T[0][0] + t21*rhs.T[1][0] + t22*rhs.T[2][0];
        T[0][1] = t00*rhs.T[0][1] + t01*rhs.T[1][1] + t02*rhs.T[2][1];
        T[1][1] = t10*rhs.T[0][1] + t11*rhs.T[1][1] + t12*rhs.T[2][1];
        T[2][1] = t20*rhs.T[0][1] + t21*rhs.T[1][1] + t22*rhs.T[2][1];
        T[0][2] = t00*rhs.T[0][2] + t01*rhs.T[1][2] + t02*rhs.T[2][2];
        T[1][2] = t10*rhs.T[0][2] + t11*rhs.T[1][2] + t12*rhs.T[2][2];
        T[2][2] = t20*rhs.T[0][2] + t21*rhs.T[1][2] + t22*rhs.T[2][2];

        return *this;
    }

    Vector2D Transform2D::translation() const{
        Vector2D newvec;
        newvec.x = T[0][2];
        newvec.y = T[1][2];
        return newvec;
    }

    double Transform2D::rotation() const{
        double newrot;
        newrot = atan2(T[1][0], T[0][0]);
        return newrot;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }


    std::ostream & operator<<(std::ostream & os, const Twist2D & v){
        os << "[" << v.thetadot << " " << v.xdot << " " << v.ydot << "]\n";
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
        std::cin >> std::ws;
        c1 = is.peek();
        if (c1 == 'd'){

            is.get(str,6);
            is >> theta;
            is.get(str,4);
            is >> vec.x;
            is.get(str,4);
            is >> vec.y;
            is.get();

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
        
        is >> v.thetadot >> v.xdot >> v.ydot;
        is.get();

        return is;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v){
        
        char c1;
        c1 = is.peek();
        if (c1 == '['){
            is.get();
        }
       
        is >> v.x >> v.y;
        is.get();

        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y << "\n";
        return os;
    }


}