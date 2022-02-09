#include <catch_ros/catch.hpp>
#include <turtlelib/rigid2d.hpp>
#include "turtlelib/diff_drive.hpp"
#include <stdio.h>
#include <iostream>
#include <sstream>


using namespace turtlelib;
static const double epsilon = 1.0e-4;


TEST_CASE("moving forward, forward kinematics", "[DiffDrive]"){ //James Avtges
    
    DiffDrive drive;
    Q zero_config;
    zero_config.x = 0;
    zero_config.y = 0;
    zero_config.theta = 0;
    Q result_config;

    Phi prev_angle;
    prev_angle.L = 0;
    prev_angle.R = 0;

    Phi next_angle;
    next_angle.L = PI/4;
    next_angle.R = PI/4;

    result_config = drive.forward_kinematics(zero_config, prev_angle, next_angle);
    CHECK(result_config.theta == Approx(0).margin(0.01));
    CHECK(result_config.x == Approx(0.033 * PI/4).margin(0.01));
    CHECK(result_config.y == Approx(0).margin(0.01));

}

TEST_CASE("moving forward, inverse kinematics", "[DiffDrive]"){ //James Avtges
    
    DiffDrive drive;
    Q zero_config;
    zero_config.x = 0;
    zero_config.y = 0;
    zero_config.theta = 0;
    Q result_config;

    Twist2D twist;
    twist.xdot = 1;
    twist.ydot = 0;
    twist.thetadot = 0;

    Phidot dd;

    dd = drive.inverse_kinematics(twist);
    CHECK(dd.Ldot == Approx(1/0.033).margin(0.01));
    CHECK(dd.Rdot == Approx(1/0.033).margin(0.01));

}


TEST_CASE("impossible twist, inverse kinematics", "[DiffDrive]"){ //James Avtges
    
    DiffDrive drive;
    Q zero_config;
    zero_config.x = 0;
    zero_config.y = 0;
    zero_config.theta = 0;
    Q result_config;

    Twist2D twist;
    twist.xdot = 0;
    twist.ydot = 1;
    twist.thetadot = 0;

    Phidot dd;

    CHECK_THROWS_AS(drive.inverse_kinematics(twist),std::logic_error);

}

TEST_CASE("turning in place, forward kinematics", "[DiffDrive]"){ //James Avtges
    
    DiffDrive drive;
    Q zero_config;
    zero_config.x = 0;
    zero_config.y = 0;
    zero_config.theta = 0;
    Q result_config;

    Phi prev_angle;
    prev_angle.L = 0;
    prev_angle.R = 0;

    Phi next_angle;
    next_angle.L = PI/4;
    next_angle.R = -PI/4;

    result_config = drive.forward_kinematics(next_angle);
    CHECK(result_config.theta == Approx(-0.033*PI/4 / 0.08).margin(0.01));
    CHECK(result_config.x == Approx(0).margin(0.01));
    CHECK(result_config.y == Approx(0).margin(0.01));

}

TEST_CASE("turning in place, inverse kinematics", "[DiffDrive]"){ //James Avtges
    
    DiffDrive drive;
    Q zero_config;
    zero_config.x = 0;
    zero_config.y = 0;
    zero_config.theta = 0;
    Q result_config;

    Twist2D twist;
    twist.xdot = 0;
    twist.ydot = 0;
    twist.thetadot = -0.033*PI/4 / 0.08;

    Phidot dd;

    dd = drive.inverse_kinematics(twist);

    CHECK(dd.Ldot == Approx(PI/4).margin(0.01));
    CHECK(dd.Rdot == Approx(-PI/4).margin(0.01));

}


TEST_CASE("moving forward and turning, forward and inverse kinematics", "[DiffDrive]"){ //James Avtges
    Twist2D twist;
    twist.xdot = PI/4;
    twist.ydot = 0;
    twist.thetadot = PI/2;
    DiffDrive drive;
    Q zero_config;
    zero_config.x = 0;
    zero_config.y = 0;
    zero_config.theta = 0;
    Q result_config;

    Phidot dd;

    double rvel, lvel;

    rvel = PI/2 * (0.5+0.08)/0.033;
    lvel = PI/2 * (0.5-0.08)/0.033;

    dd = drive.inverse_kinematics(twist);
    CHECK(dd.Ldot == Approx(lvel).margin(0.01));
    CHECK(dd.Rdot == Approx(rvel).margin(0.01));

    result_config = drive.forward_kinematics(zero_config, twist);
    CHECK(result_config.theta == Approx(PI/2).margin(0.01));
    CHECK(result_config.x == Approx(0.5).margin(0.01));
    CHECK(result_config.y == Approx(0.5).margin(0.01));

}


TEST_CASE("istream Vector input","[Vector2D]"){ // James Avtges
    std::stringstream bracket;
    Vector2D bracketV;
    bracket.str("[1 1]");
    std::stringstream number;
    Vector2D numberV;
    number.str("1 1");
    bracket >> bracketV;
    number >> numberV;
    CHECK(bracketV.x == 1);
    CHECK(numberV.x == 1);
}

TEST_CASE("ostream Vector output","[Vector2D]"){ // James Avtges
    std::stringstream vectorOut;
    Vector2D vector;
    vector.x = 9;
    vector.y = 1;

    vectorOut << vector;

    CHECK(vectorOut.str() == "[9 1]");
}

TEST_CASE("istream Twist input","[Twist2D]"){ // James Avtges
    std::stringstream bracket;
    Twist2D bracketT;
    bracket.str("[1 2 3]");
    std::stringstream number;
    Twist2D numberT;
    number.str("1 2 3");
    bracket >> bracketT;
    number >> numberT;
    CHECK(bracketT.xdot == 2);
    CHECK(numberT.xdot == 2);
}

TEST_CASE("ostream Twist output","[Twist2D]"){ // James Avtges
    std::stringstream twistOut;
    Twist2D twist;
    twist.thetadot = 10;
    twist.xdot = 5;
    twist.ydot = 4;

    twistOut << twist;

    CHECK(twistOut.str() == "[10 5 4]");
}

TEST_CASE("istream Transform input","[Transform2D]"){ // James Avtges
    std::stringstream transform;
    Transform2D T(0);
    transform.str("deg: 80 x: 2 y: 4\n");
    transform >> T;
    
    Vector2D translation = T.translation();
    double rotation = rad2deg(T.rotation());
    CHECK(translation.x == 2);
    CHECK(translation.y == 4);
    CHECK(rotation == 80);
}

TEST_CASE("istream Transform input, Matt's test","[Transform2D]"){ // James Avtges (Matt Elwin)
    std::stringstream transform;
    Transform2D tfin;
    transform << "45 -1 7";
    transform >> tfin;
    
    Vector2D translation = tfin.translation();
    double rotation = tfin.rotation();
    CHECK(translation.x == -1);
    CHECK(translation.y == 7);
    CHECK(rotation == PI/4);
}

TEST_CASE("ostream Transform output","[Transform2D]"){ // James Avtges
    std::stringstream transformOut;
    Vector2D trans;
    trans.x = 3.2;
    trans.y = 4;
    double rot_deg = 6.1;
    Transform2D T(trans,deg2rad(rot_deg));

    transformOut << T;

    CHECK(transformOut.str() == "deg: 6.1 x: 3.2 y: 4\n");
}

/* TESTS FROM ANNA */

TEST_CASE("constructor_all", "[transform]") { // Anna Garverick
    Vector2D v;
    v.x = 1;
    v.y = 2;

    double r = PI/2;

    Transform2D T(v, r);

    Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = r_out;

    CHECK(t_out.x == 1);
    CHECK(t_out.y == 2);
    CHECK(d == PI/2);
} 

TEST_CASE("constructor_trans", "[transform]") { // Anna Garverick
    Vector2D v;
    v.x = 1;
    v.y = 2;

    Transform2D T(v);

    Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = r_out;

    CHECK(t_out.x == 1);
    CHECK(t_out.y == 2);
    CHECK(d == 0);
}

TEST_CASE("constructor_rot", "[transform]") { // Anna Garverick
    double r = PI/2;

    Transform2D T(r);

    Vector2D t_out = T.translation();
    double r_out = T.rotation();

    double d = r_out;

    CHECK(t_out.x == 0);
    CHECK(t_out.y == 0);
    CHECK(d == PI/2);
}
TEST_CASE("inv", "[inverse]") { // Anna Garverick
    Vector2D v;
    v.x = 1;
    v.y = 1;

    double r = PI/4;

    Transform2D T(v,r);

    Transform2D T_inv(0);
    T_inv = T.inv();

    Vector2D t_out = T_inv.translation();
    double r_out = T_inv.rotation();

    CHECK(t_out.x == Approx(-1.41421).margin(.01));
    CHECK(t_out.y == Approx(0.0).margin(.01));
    CHECK(r_out == Approx(-1*PI/4).margin(.01));
}

TEST_CASE("trans", "[translation]") { //Anna Garverick
    Vector2D v;
    v.x = 5;
    v.y = 10;

    Transform2D T(v);

    Vector2D t_out = T.translation();

    CHECK(t_out.x == 5);
    CHECK(t_out.y == 10);
}

TEST_CASE("rot", "[rotation]") { //Anna Garverick
    double r = PI/4;

    Transform2D T(r);

    double r_out = T.rotation();

    double d = r_out;

    CHECK(d == Approx(PI/4).margin(.001));
}


TEST_CASE("normalize"){ // Cody Nichoson
    Vector2D v_in, v_out;
    v_in.x = 1; 
    v_in.y = 1;
    v_out = normalize(v_in);
    CHECK(v_out.x == Approx(0.707).margin(0.001));
    CHECK(v_out.y == Approx(0.707).margin(0.001));
}

TEST_CASE("normalize_angle", "[Transform2D]"){ // James Avtges
    CHECK(normalize_angle(PI) == Approx(PI).margin(0.001));
    CHECK(normalize_angle(-1*PI) == Approx(PI).margin(0.001));
    CHECK(normalize_angle((-1*PI)+0.0001) == Approx((-1*PI)+0.0001).margin(0.001));
    CHECK(normalize_angle(0) == Approx(0).margin(0.001));
    CHECK(normalize_angle(-1*PI/4) == Approx(-1*PI/4).margin(0.001));
    CHECK(normalize_angle(3*PI/2) == Approx(-1*PI/2).margin(0.001));
    CHECK(normalize_angle(-5*PI/2) == Approx(-1*PI/2).margin(0.001));
}

TEST_CASE("twist integration", "[Transform2D]"){ // James Avtges
    Transform2D test_transform(0), res1(0), res2(0), res3(0);
    Twist2D rotation, translation, both;
    rotation.thetadot = PI/2;
    translation.xdot = 3;
    translation.ydot = 2;
    both.thetadot = PI/2;
    both.xdot = 1;
    both.ydot = 1;

    res1 = integrate_twist(rotation);
    res2 = integrate_twist(translation);
    res3 = integrate_twist(both);

    CHECK(res1.rotation() == Approx(PI/2).margin(epsilon));

    CHECK(res2.translation().x == Approx(3).margin(epsilon));
    CHECK(res2.translation().y == Approx(2).margin(epsilon));

    CHECK(res3.rotation() == Approx(PI/2).margin(epsilon));
    CHECK(res3.translation().x == Approx(0).margin(epsilon));
    CHECK(res3.translation().y == Approx(4/PI).margin(epsilon));

}

TEST_CASE("Integrating a Twist", "[rigid2d]") { // Marco Morales & RKS
    Twist2D twist;
    Transform2D TbbPrime(0);
    Vector2D trans;
    double rot;
    SECTION( "Testing Both Rotational and Translational components v2" ) {
        twist.xdot = 1.0;
        twist.ydot = 2.0;
        twist.thetadot = PI;
        TbbPrime = integrate_twist(twist);
        trans = TbbPrime.translation();
        rot = TbbPrime.rotation();
        CHECK( trans.x == Approx(-1.27324));
        CHECK( trans.y ==  Approx(0.63662));
        CHECK( rot ==  Approx(PI));
    }

    SECTION( "Testing Both Rotational and Translational components v3" ) {
        twist.xdot = 2.0;
        twist.ydot = 4.0;
        twist.thetadot = PI;
        TbbPrime = integrate_twist(twist);
        trans = TbbPrime.translation();
        rot = TbbPrime.rotation();
        CHECK( trans.x == Approx(-2.54).margin(0.01));
        CHECK( trans.y ==  Approx(1.272).margin(0.01));
        CHECK( rot ==  Approx(PI));
    }

    SECTION( "Testing Both Rotational and Translational components v4" ) {
        twist.xdot = 0.0;
        twist.ydot = 0.0;
        twist.thetadot = 0;
        TbbPrime = integrate_twist(twist);
        trans = TbbPrime.translation();
        rot = TbbPrime.rotation();
        CHECK( trans.x == Approx(0.0).margin(0.01));
        CHECK( trans.y ==  Approx(0.0).margin(0.01));
        CHECK( rot ==  Approx(0.0).margin(0.01));
    }

    SECTION( "Testing Both Rotational and Translational components v5" ) {
        twist.xdot = 5.0;
        twist.ydot = 2.0;
        twist.thetadot = 0.0;
        TbbPrime = integrate_twist(twist);
        trans = TbbPrime.translation();
        rot = TbbPrime.rotation();
        CHECK( trans.x == Approx(5.0).margin(0.01));
        CHECK( trans.y ==  Approx(2.0).margin(0.01));
        CHECK( rot ==  Approx(0).margin(0.01));
    }

    SECTION( "Testing Both Rotational and Translational components v6" ) {
        twist.xdot = 0.0;
        twist.ydot = 0.0;
        twist.thetadot = PI/4;
        TbbPrime = integrate_twist(twist);
        trans = TbbPrime.translation();
        rot = TbbPrime.rotation();
        CHECK( trans.x == Approx(0.0).margin(0.01));
        CHECK( trans.y ==  Approx(0.0).margin(0.01));
        CHECK( rot ==  Approx(PI/4));
    }

    SECTION( "Testing Both Rotational and Translational components v7" ) {
        twist.xdot = 5.0;
        twist.ydot = 2.0;
        twist.thetadot = PI/4;
        TbbPrime = integrate_twist(twist);
        trans = TbbPrime.translation();
        rot = TbbPrime.rotation();
        CHECK( trans.x == Approx(3.75574).margin(0.01));
        CHECK( trans.y ==  Approx(3.66525).margin(0.01));
        CHECK( rot ==  Approx(PI/4));
    }
}

////////////////// TRANSFORM2D ////////////

/// TEST operator*= ///
TEST_CASE("Transform2D::operator*=, Self Reference", "[Transform2D]") // RKS
{
    //Build objects
    Transform2D T_id{0};
    Vector2D vec_id{0,0};

    //Perform functions
    T_id *= T_id;

    //Grab results
    Vector2D res_vec = T_id.translation();
    double res_ang = T_id.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Identity Mul", "[Transform2D]") //RKS
{
    //Build objects
    Transform2D T_id{0};
    Transform2D T_id2{0};
    Vector2D vec_id{0,0};

    //Perform functions
    T_id *= T_id2;

    //Grab results
    Vector2D res_vec = T_id.translation();
    double res_ang = T_id.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}



TEST_CASE("Transform2D::operator*=, Cancelling Rotations", "[Transform2D]") //RKS
{
    //Build objects
    Transform2D T_a{-PI/4};
    Transform2D T_b{PI/4};
    Vector2D vec_id{0,0};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Cancelling Transitions", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{1,2};
    Vector2D vec_b{-1,-2};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{0,0};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Only Transitions", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{2,2};
    Vector2D vec_b{1,3};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{3,5};

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Basic Transformation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = PI/4;
    Vector2D vec_b{2,3};
    double ang_b = PI/3;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{0.292893,5.53553};
    double ang_ans = 1.8326;

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = T_a.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    CHECK(res_ang == Approx( ang_ans ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*=, Big Rotation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = 4.88692;
    Vector2D vec_b{-2,3};
    double ang_b = 1.74533;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{3.60713, 4.49056};
    double ang_ans = 0.349066;

    //Perform functions
    T_a *= T_b;

    //Grab results
    Vector2D res_vec = T_a.translation();
    double res_ang = std::fmod(T_a.rotation(), 2.0*PI);

    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    CHECK(res_ang == Approx( ang_ans ).margin(epsilon));
}


/// TEST operator* ///

TEST_CASE("Transform2D::operator*, Identity Mul", "[Transform2D]") // RKS
{
    //Build objects
    Transform2D T_id{0};
    Transform2D T_id2{0};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_id3 = T_id * T_id2;

    //Grab results
    Vector2D res_vec = T_id3.translation();
    double res_ang = T_id3.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}



TEST_CASE("Transform2D::operator*, Cancelling Rotations", "[Transform2D]") // RKS
{
    //Build objects
    Transform2D T_a{-PI/4};
    Transform2D T_b{PI/4};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Cancelling Transitions", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{1,2};
    Vector2D vec_b{-1,-2};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{0,0};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Only Transitions", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{2,2};
    Vector2D vec_b{1,3};
    Transform2D T_a{vec_a};
    Transform2D T_b{vec_b};
    Vector2D vec_id{3,5};

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_id.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_id.y ).margin(epsilon));
    CHECK(res_ang == Approx( 0.0 ).margin(epsilon));
}

TEST_CASE("Transform2D::operator*, Basic Transformation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = PI/4;
    Vector2D vec_b{2,3};
    double ang_b = PI/3;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{0.292893,5.53553};
    double ang_ans = 1.8326;

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = T_c.rotation();

    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    CHECK(res_ang == Approx( ang_ans ).margin(epsilon));
}



TEST_CASE("Transform2D::operator*, Big Rotation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{1,2};
    double ang_a = 4.88692;
    Vector2D vec_b{-2,3};
    double ang_b = 1.74533;
    Transform2D T_a{vec_a, ang_a};
    Transform2D T_b{vec_b, ang_b};

    // Answer
    Vector2D vec_ans{3.60713, 4.49056};
    double ang_ans = 0.349066;

    //Perform functions
    Transform2D T_c = T_a * T_b;

    //Grab results
    Vector2D res_vec = T_c.translation();
    double res_ang = std::fmod(T_c.rotation(), 2.0*PI);

    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
    CHECK(res_ang == Approx( ang_ans ).margin(epsilon));
}

/// TEST operator()(Vector2D) ///

TEST_CASE("Transform2D::operator(Vector), Idenity Transform", "[Transform2D]") // RKS
{
    //Build objects
    Transform2D T_ab{0};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{1.0, 1.0};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);

    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Rotation", "[Transform2D]") // RKS
{
    //Build objects
    Transform2D T_ab{PI/4};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{0.0, 1.414213};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Translation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{3.0, -1.0};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Vector), Simple Transformation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a, PI/4};
    Vector2D vec_b{1.0, 1.0};

    // Answer
    Vector2D vec_ans{2., -0.58578};

    //Perform functions
    Vector2D res_vec = T_ab(vec_b);
    
    // Tests
    CHECK(res_vec.x == Approx( vec_ans.x ).margin(epsilon));
    CHECK(res_vec.y == Approx( vec_ans.y ).margin(epsilon));
}

/// TEST operator()(Twist2D) ///

TEST_CASE("Transform2D::operator(Twist2D), Idenity Transform", "[Transform2D]") // RKS
{
    //Build objects
    Transform2D T_ab{0};
    Twist2D twt_b{30, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{30, 1.0, 2.0};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    CHECK(res_twt.thetadot == Approx ( twt_ans.thetadot).margin(epsilon));
    CHECK(res_twt.xdot == Approx( twt_ans.xdot ).margin(epsilon));
    CHECK(res_twt.ydot == Approx( twt_ans.ydot ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Rotation", "[Transform2D]") // RKS
{
    //Build objects
    Transform2D T_ab{PI/4};
    Twist2D twt_b{PI/6, 1.0, 2.0};

    // Answer
    Twist2D twt_ans{PI/6, -0.70710678, 2.12132034};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    CHECK(res_twt.thetadot == Approx ( twt_ans.thetadot).margin(epsilon));
    CHECK(res_twt.xdot == Approx( twt_ans.xdot ).margin(epsilon));
    CHECK(res_twt.ydot == Approx( twt_ans.ydot ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Translation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a};
    Twist2D twt_b{deg2rad(30), 1.0, 2.0};

    // Answer
    Twist2D twt_ans{deg2rad(30), -0.04719755, 0.95280245};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);
    std::cout << res_twt;

    // Tests
    CHECK(res_twt.thetadot == Approx ( (twt_ans.thetadot)).margin(epsilon));
    CHECK(res_twt.xdot == Approx( twt_ans.xdot ).margin(epsilon));
    CHECK(res_twt.ydot == Approx( twt_ans.ydot ).margin(epsilon));
}

TEST_CASE("Transform2D::operator(Twist2D), Simple Transformation", "[Transform2D]") // RKS
{
    //Build objects
    Vector2D vec_a{2.0, -2.0};
    Transform2D T_ab{vec_a, PI/4};
    Twist2D twt_b{deg2rad(30), 1.0, 2.0};

    // Answer
    Twist2D twt_ans{deg2rad(30), -1.75430433, 1.07412279};

    //Perform functions
    Twist2D res_twt = T_ab(twt_b);

    // Tests
    CHECK(res_twt.thetadot == Approx ( (twt_ans.thetadot)).margin(epsilon));
    CHECK(res_twt.xdot == Approx( twt_ans.xdot ).margin(epsilon));
    CHECK(res_twt.ydot == Approx( twt_ans.ydot ).margin(epsilon));
}
