#include <stdio.h>
#include <iostream>
#include "turtlelib/rigid2d.hpp"
/// \file
/// \brief A main file for inputting, calculating various 2D Transformations.

using namespace std;

/// The main loop of the program.
int main(void){

    /// Defines the Transform2D, Vector2D, and Twist2D objects that will be used in this program.
    turtlelib::Transform2D Tab(0), Tbc(0), Tba(0), Tcb(0), Tac(0), Tca(0);
    turtlelib::Vector2D v_b, v_bhat, v_c, v_a, test;
    turtlelib::Twist2D V_a, V_b, V_c;

    // cout << "test test" << endl;
    // cin >> test;
    // cout << test + test;
    // cout << test - test;
    // cout << test * 4;
    // cout << 4 * test;

    /// Inputting Transform2D objects, and calculating other transfroms from these two
    cout << "Enter transform T_{a,b}:" << endl;
    cin >> Tab;
    cout << "Enter transform T_{b,c}:" << endl;
    cin >> Tbc;
    cout << "T_{a_b}: " << Tab;
    Tba = Tab.inv();
    cout << "T_{b_a}: " << Tba;
    cout << "T_{b_c}: " << Tbc;
    Tcb = Tbc.inv();
    cout << "T_{c_b}: " << Tcb;

    Tac = Tab * Tbc;
    cout << "T_{a_c}: " << Tac;
    Tca = Tac.inv();
    cout << "T_{c_a}: " << Tca;

    /// Inputting Vector2D objects, and calculating other vectors using the 2D transforms
    cout << "Enter vector v_b:" << endl;
    cin >> v_b;
    v_bhat = normalize(v_b);
    v_a = Tab(v_b);
    v_c = Tcb(v_b);

    cout << "v_bhat: " << v_bhat;
    cout << "v_a: " << v_a << "\n";
    cout << "v_b: " << v_b << "\n";
    cout << "v_c: " << v_c << "\n";

    /// Inputting Twist2D objects, and calculating other twists using the 2D transforms
    cout << "Enter twist V_b:" << endl;
    cin >> V_b;
    V_a = Tab(V_b);
    V_c = Tcb(V_b);
    cout << "V_a " << V_a << "\n";
    cout << "V_b " << V_b << "\n";
    cout << "V_c " << V_c << "\n";
    cout << "AAAAA" << endl;

    //  TTest(0);
    turtlelib::Transform2D TTest = integrate_twist(V_b);
    cout << TTest;

}