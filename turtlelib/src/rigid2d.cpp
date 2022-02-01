#include <iostream>
#include <string>
#include "turtlelib/rigid2d.hpp"
#include <cmath>
/// \file
/// \brief The corresponding cpp file for "rigid2d.hpp", for two-dimensional rigid body transformations.


namespace turtlelib{


    /// \brief defines a Transform2D object with a Vector2D translation
    /// \param trans - input Vector2D with x and y translational components
    Transform2D::Transform2D(){
        T[0][0] = 1;
        T[1][0] = 0;
        T[2][0] = 0;
        T[0][1] = 0;
        T[1][1] = 1;
        T[2][1] = 0;
        T[0][2] = 0;
        T[1][2] = 0;
        T[2][2] = 1;
    }
    
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

    /// \brief defines a Transform2D object with a double degrees rotation only
    /// \param degrees - input double of degrees
    Transform2D::Transform2D(double radians){

        T[0][0] = cos(radians);
        T[1][0] = sin(radians);
        T[2][0] = 0;
        T[0][1] = -sin(radians);
        T[1][1] = cos(radians);
        T[2][1] = 0;
        T[0][2] = 0;
        T[1][2] = 0;
        T[2][2] = 1;

    }

    /// \brief defines a Transform2D object with a Vector2D rotation and double degrees rotation
    /// \param trans - input Vector2D with x and y translational components
    /// \param degrees - input double of degrees
    Transform2D::Transform2D(Vector2D trans, double radians){

        T[0][0] = cos(radians);
        T[1][0] = sin(radians);
        T[2][0] = 0;
        T[0][1] = -sin(radians);
        T[1][1] = cos(radians);
        T[2][1] = 0;
        T[0][2] = trans.x;
        T[1][2] = trans.y;
        T[2][2] = 1;

    }


    /// \brief apply a transformation to a Vector2D
    /// \param v - the vector to transform
    /// \return a vector in the new coordinate system
    Vector2D Transform2D::operator()(Vector2D v) const{

        Vector2D outVec;
        outVec.x = T[0][0]*v.x + T[0][1]*v.y + T[0][2]*1;
        outVec.y = T[1][0]*v.x + T[1][1]*v.y + T[1][2]*1;

        return outVec;
    }

    /// \brief normalize a Vector2D object
    /// \param v - the vector to normalize
    /// \return a normalized Vector2D object with a magnitude of 1 (unit vector)
    Vector2D normalize(Vector2D v){
        Vector2D output;
        
        double mag = sqrt(v.x * v.x + v.y * v.y);
        output.x = v.x/mag;
        output.y = v.y/mag;

        return output;
    }

    double normalize_angle(double rad){
        long double d = std::fmod(rad-PI, 2*PI);
        if (d > 0){
            d -= 2*PI;
        }
        return d + PI;
    }

    /// \brief apply a transformation to a Twist2D
    /// \param v - the twist to transform
    /// \return a twist in the new coordinate system
    Twist2D Transform2D::operator()(Twist2D v) const{

        Twist2D outTwist;
        outTwist.thetadot = 0 * v.xdot + 0 * v.ydot + v.thetadot;
        outTwist.xdot = T[1][2]*v.thetadot + T[0][0]*v.xdot + T[0][1]*v.ydot;
        outTwist.ydot = -1*T[0][2] * v.thetadot + T[1][0] * v.xdot + T[1][1] * v.ydot;

        return outTwist;
    }

    /// \brief invert the Transformation2D object
    /// \return the inverse transformation 
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

    /// \brief compose this transform with another and store the result 
    /// in this object
    /// \param rhs - the first transform to apply
    /// \return a reference to the newly transformed operator
    Transform2D & Transform2D::operator*=(const Transform2D & rhs){

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

    Vector2D & Vector2D::operator+=(const Vector2D & rhs){

        this->x += rhs.x;
        this->y += rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator-=(const Vector2D & rhs){

        this->x -= rhs.x;
        this->y -= rhs.y;
        return *this;
    }

    Vector2D & Vector2D::operator*=(const double & rhs){

        this->x *= rhs;
        this->y *= rhs;
        return *this;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs){
        return lhs+=rhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs){
        return lhs-=rhs;
    }

    Vector2D operator*(Vector2D lhs, const double & rhs){
        return lhs*=rhs;
    }

    Vector2D operator*(const double & lhs, Vector2D rhs){
        return rhs*=lhs;
    }

    double dot(Vector2D lhs, Vector2D rhs){
        double dot_product;
        dot_product = (lhs.x + rhs.x) + (lhs.y + rhs.y);
        return dot_product;
    }

    double magnitude(Vector2D vec){
        return sqrt(vec.x*vec.x + vec.y*vec.y);
    }

    double angle(Vector2D lhs, Vector2D rhs){
        return acos(dot(lhs,rhs) / magnitude(lhs) * magnitude(rhs));
    }

    /// \brief the translational component of the transform
    /// \return the x,y translation
    Vector2D Transform2D::translation() const{
        Vector2D newvec;
        newvec.x = T[0][2];
        newvec.y = T[1][2];
        return newvec;
    }

    /// \brief get the angular displacement of the transform
    /// \return the angular displacement, in radians
    double Transform2D::rotation() const{
        double newrot;
        newrot = atan2(T[1][0], T[0][0]);
        return newrot;
    }


    Transform2D integrate_twist(Twist2D twist){
        double theta;
        Vector2D d;
        

        // theta = thetadot from twist
        // thetadot * -ys + xdot = 0
        // thetadot * xs + ydot = 0

        if (twist.thetadot == 0){
            d.y = twist.ydot;
            d.x = twist.xdot;

            Transform2D Tbb(d);
            return Tbb;
        }

        else {
            d.x = twist.ydot / twist.thetadot;
            d.y = -1 * twist.xdot / twist.thetadot; // May be an issue
            theta = twist.thetadot;

            Transform2D Tss(theta);
            Transform2D Tsb(d);
            Transform2D Tbs = Tsb.inv();

            Transform2D Tbb = (Tbs * Tss) * Tsb;

            return Tbb;
        }

    }

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        return lhs*=rhs;
    }

    /// \brief prints a human readable version of the Twist:
    /// An example output:
    /// [1 2 3]
    /// \param os - an output stream
    /// \param tf - the twist to print
    std::ostream & operator<<(std::ostream & os, const Twist2D & v){
        os << "[" << v.thetadot << " " << v.xdot << " " << v.ydot << "]";
        return os;
    }

    /// \brief prints a human readable version of the Vector2D:
    /// An example output:
    /// [1 3]
    /// \param os - an output stream
    /// \param tf - the vector to print
    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]";
        return os;
    }

    /// \brief Read a transformation from stdin
    /// \param is - an input stream
    /// \param tf - the transform to input
    /// An example input:
    /// deg: 90 x: 3 y: 5
    std::istream & operator>>(std::istream & is, Transform2D & tf){
        Vector2D vec;
        double theta;
        char str[6];
        char c1;
        c1 = is.peek();
        if (c1 == 'd'){

            is.get(str,6);
            is >> theta;
            theta = deg2rad(theta);
            is.get(str,4);
            is >> vec.x;
            is.get(str,4);
            is >> vec.y;
            is.get();

            Transform2D output(vec,theta);
            tf = output;
        }
        else{
            is >> theta >> vec.x >> vec.y;
            Transform2D output(vec,deg2rad(theta));
            tf = output;
        }

        return is;
    }

    /// \brief Read a Twist2D from stdin
    /// \param is - an input stream
    /// \param tf - the twist to input
    /// An example input:
    /// [1 2 3] or 1 2 3
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

    /// \brief Read a Vector2D from stdin
    /// \param is - an input stream
    /// \param tf - the vector to input
    /// An example input:
    /// [1 2] or 1 2
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

    /// \brief prints a human readable version of the transform:
    /// An example output:
    /// deg: 90 x: 3 y: 5
    /// \param os - an output stream
    /// \param tf - the transform to print
    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        os << "deg: " << rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y << "\n";
        return os;
    }


}