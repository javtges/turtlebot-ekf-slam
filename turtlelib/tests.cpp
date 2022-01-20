#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "rigid2d.hpp"
#include <stdio.h>
#include <iostream>
#include <sstream>


TEST_CASE("istream Vector input","[Vector2D]"){ // James Avtges
    std::stringstream bracket;
    turtlelib::Vector2D bracketV;
    bracket.str("[1 1]");
    std::stringstream number;
    turtlelib::Vector2D numberV;
    number.str("1 1");
    bracket >> bracketV;
    number >> numberV;
    REQUIRE(bracketV.x == 1);
    REQUIRE(numberV.x == 1);
}

TEST_CASE("ostream Vector output","[Vector2D]"){ // James Avtges
    std::stringstream vectorOut;
    turtlelib::Vector2D vector;
    vector.x = 9;
    vector.y = 1;

    vectorOut << vector;

    REQUIRE(vectorOut.str() == "[9 1]\n");
}

TEST_CASE("istream Twist input","[Twist2D]"){ // James Avtges
    std::stringstream bracket;
    turtlelib::Twist2D bracketT;
    bracket.str("[1 2 3]");
    std::stringstream number;
    turtlelib::Twist2D numberT;
    number.str("1 2 3");
    bracket >> bracketT;
    number >> numberT;
    REQUIRE(bracketT.xdot == 2);
    REQUIRE(numberT.xdot == 2);
}

TEST_CASE("ostream Twist output","[Twist2D]"){ // James Avtges
    std::stringstream twistOut;
    turtlelib::Twist2D twist;
    twist.thetadot = 10;
    twist.xdot = 5;
    twist.ydot = 4;

    twistOut << twist;

    REQUIRE(twistOut.str() == "[10 5 4]\n");
}

TEST_CASE("istream Transform input","[Transform2D]"){ // James Avtges
    std::stringstream transform;
    turtlelib::Transform2D T(0);
    transform.str("deg: 80 x: 2 y: 4\n");
    transform >> T;
    
    turtlelib::Vector2D translation = T.translation();
    double rotation = turtlelib::rad2deg(T.rotation());
    REQUIRE(translation.x == 2);
    REQUIRE(translation.y == 4);
    REQUIRE(rotation == 80);
}

TEST_CASE("ostream Transform output","[Transform2D]"){ // James Avtges
    std::stringstream transformOut;
    turtlelib::Vector2D trans;
    trans.x = 3.2;
    trans.y = 4;
    double rot = 6.1;
    turtlelib::Transform2D T(trans,rot);

    transformOut << T;

    REQUIRE(transformOut.str() == "deg: 6.1 x: 3.2 y: 4\n");
}

/* TESTS FROM ANNA */

TEST_CASE("constructor_all", "[transform]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    double r = 90;

    turtlelib::Transform2D T(v, r);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 1);
    REQUIRE(t_out.y == 2);
    REQUIRE(d == 90);
} 

TEST_CASE("constructor_trans", "[transform]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    turtlelib::Transform2D T(v);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 1);
    REQUIRE(t_out.y == 2);
    REQUIRE(d == 0);
}

TEST_CASE("constructor_rot", "[transform]") { // Anna Garverick
    double r = 90;

    turtlelib::Transform2D T(r);

    turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 0);
    REQUIRE(t_out.y == 0);
    REQUIRE(d == 90);
}
TEST_CASE("inv", "[inverse]") { // Anna Garverick
    turtlelib::Vector2D v;
    v.x = 1;
    v.y = 2;

    double r = 45;

    turtlelib::Transform2D T(v,r);

    turtlelib::Transform2D T_inv(0);
    T_inv = T.inv();

    turtlelib::Vector2D t_out = T_inv.translation();
    double r_out = T_inv.rotation();

    REQUIRE(t_out.x == Approx(-2.121).margin(.01));
    REQUIRE(t_out.y == Approx(-0.7071).margin(.01));
    REQUIRE(r_out == Approx(-1*turtlelib::PI/4).margin(.01));
}

TEST_CASE("trans", "[translation]") { //Anna Garverick
    turtlelib::Vector2D v;
    v.x = 5;
    v.y = 10;

    turtlelib::Transform2D T(v);

    turtlelib::Vector2D t_out = T.translation();
    // double r_out = T.rotation();

    // double d = turtlelib::rad2deg(r_out);

    REQUIRE(t_out.x == 5);
    REQUIRE(t_out.y == 10);
}

TEST_CASE("rot", "[rotation]") { //Anna Garverick
    double r = 33;

    turtlelib::Transform2D T(r);

    // turtlelib::Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = turtlelib::rad2deg(r_out);

    REQUIRE(d == Approx(33).margin(.001));
}