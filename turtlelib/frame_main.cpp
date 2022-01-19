#include <stdio.h>
#include <iostream>
#include "rigid2d.hpp"

using namespace std;


int main(void){

    turtlelib::Transform2D Tab(0), Tbc(0), Tba(0), Tcb(0), Tac(0), Tca(0);
    turtlelib::Vector2D v_b, v_bhat, v_c, v_a;
    turtlelib::Twist2D V_a, V_b, V_c;

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

    cout << "Enter vector v_b:" << endl;
    cin >> v_b;
    v_bhat = Tab.normalize(v_b);
    v_a = Tab(v_b);
    v_c = Tcb(v_b);


    cout << "v_bhat: " << v_bhat;
    cout << "v_a: " << v_a;
    cout << "v_b: " << v_b;
    cout << "v_c: " << v_c;


    cout << "Enter twist V_b:" << endl;
    cin >> V_b;
    V_a = Tab(V_b);
    V_c = Tcb(V_b);
    cout << "V_a " << V_a;
    cout << "V_b " << V_b;
    cout << "V_c " << V_c;



}