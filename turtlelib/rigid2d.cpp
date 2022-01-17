#include <iostream>
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

    Vector2D operator()(Vector2D v) const{

        float v_arr[3][1];
        v_arr[0][0] = v.x;
        v_arr[0][0] = v.y;
        v_arr[0][0] = 1;

        Vector2D outVec;
        outVec.x = T[0][0]*v.x + T[0][1]*v.y + T[0][2]*1;
        outVec.y = T[1][0]*v.x + T[1][1]*v.y + T[1][2]*1;

        return outVec;
    }


    Transform2D inv() const{
        Transform2D Tinv;

        Tinv.T[0][0] = T[0][0];
        Tinv.T[1][0] = T[0][1];
        Tinv.T[2][0] = 0;
        Tinv.T[0][1] = T[1][0];
        Tinv.T[1][1] = T[1][1];
        Tinv.T[2][1] = 0;
        Tinv.T[0][2] = -1*T[0][2];
        Tinv.T[1][2] = -1*T[1][2];
        Tinv.T[2][2] = 1;

    }



}