#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/ekf.hpp"
#include <sstream>
#include <string>
#include <cmath>
#include <armadillo>

using turtlelib::PI;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::Vector2D;
using turtlelib::DiffDrive;
using turtlelib::Wheel_Vel;
using turtlelib::Config;
using std::stringstream;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

static constexpr auto zero_margin = std::numeric_limits<double>::epsilon()*100;

TEMPLATE_TEST_CASE("Normalize rotation angle", "[normalize]", double){
    double rad1{};
    double rad2{};

    SECTION("rad = PI"){
        rad1 = PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(PI));
    }

    SECTION("rad = 3PI"){
        rad1 = 3*PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(-PI));
    }

    SECTION("rad = 3PI"){
        rad1 = 7*PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(PI));
    }

    SECTION("rad = -PI"){
        rad1 = -1.0*PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(-PI));
    }
    
    SECTION("rad = 0"){
        rad1 = 0.0;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(0.0));
    }

    SECTION("rad = -PI/4"){
        rad1 = -1.0/4.0 * PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(-1.0/4.0 * PI));
    }

    SECTION("rad = 3*PI/2"){
        rad1 = 3.0/2.0 * PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(1.0/2.0 * PI) || WithinRel(-1.0/2.0*PI));
    }

    SECTION("rad = -5*PI/2"){
        rad1 = -5.0/2.0 * PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(-1.0/2.0 * PI));
    }

    SECTION("rad = -5*PI/2"){
        rad1 = -3.5;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinAbs(2.7831, .0001));
    }

    SECTION("rad = -3PI"){
        rad1 = -3*PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinAbs(PI, .0001));
    }

}



TEST_CASE("An identity matrix is constructed", "[transform]"){
    const Transform2D T{};

    CHECK_THAT(T.rotation(), WithinAbs(0.0, zero_margin));
    CHECK_THAT(T.translation().x, WithinAbs(0.0, zero_margin));
    CHECK_THAT(T.translation().y, WithinAbs(0.0, zero_margin));
}

TEST_CASE("Constructor - Pure Translation", "[transform]"){
    const turtlelib::Vector2D vec{25.0, -19.0};
    const Transform2D T{vec};

    CHECK_THAT(T.rotation(), WithinAbs(0.0, zero_margin));
    CHECK_THAT(T.translation().x, WithinRel(25.0));
    CHECK_THAT(T.translation().y, WithinRel(-19.0));
}


TEST_CASE("Constructor - Pure Rotation", "[transform]"){
    const Transform2D T{PI};

    CHECK_THAT(T.rotation(), WithinRel(-PI) || WithinRel(PI));
    CHECK_THAT(T.translation().x, WithinAbs(0.0, zero_margin));
    CHECK_THAT(T.translation().y, WithinAbs(0.0, zero_margin));
}

TEST_CASE("Constructor - Transform", "[transform]"){
    const Transform2D T{{18.0,-1.0},PI};

    CHECK_THAT(T.rotation(), WithinRel(-PI) || WithinRel(PI));
    CHECK_THAT(T.translation().x, WithinRel(18.0));
    CHECK_THAT(T.translation().y, WithinRel(-1.0));
}

TEST_CASE("Operator - () ", "[operator]"){
    const Transform2D T{{18.0,-1.0},PI};
    const Vector2D test_vec{T({1.0,1.0})};

    CHECK_THAT(test_vec.x, WithinRel(17.0));
    CHECK_THAT(test_vec.y, WithinRel(-2.0));
}

TEST_CASE(" Inverse transform 1", "[transform]"){
    const Transform2D T{{1.0,0.0},1.0};

    CHECK_THAT(T.inv().rotation(), WithinRel(-1.0));
    CHECK_THAT(T.inv().translation().x, WithinRel(-0.5403, 0.0001));
    CHECK_THAT(T.inv().translation().y, WithinRel(0.84147, 0.0001));
}

TEST_CASE(" Inverse transform 2", "[transform]"){
    const Transform2D T{{4,2},3.5};

    CHECK_THAT(T.inv().rotation(), WithinRel(2.7831, 0.0001) || WithinRel(-0.3584, 0.0001));
    CHECK_THAT(T.inv().translation().x, WithinRel(4.4474, 0.0001));
    CHECK_THAT(T.inv().translation().y, WithinRel(0.4698, 0.0001));
}

TEMPLATE_TEST_CASE("Operator - *= ", "[operator]", Transform2D){
    
    SECTION("Test 1"){
        Transform2D T1{{-1.0,2.0},PI/4.0};
        const Transform2D T2{{3.0,5.0},-3.0*PI/2.0};
        T1*=T2;

        CHECK_THAT(T1.rotation(),WithinRel(3*PI/4.0, .0001));
        CHECK_THAT(T1.translation().x, WithinRel(-1.0-sqrt(2)));
        CHECK_THAT(T1.translation().y,WithinRel(2+4*sqrt(2)));
    }

    SECTION("Test 2"){
        Transform2D T1{{4.0,-2.0},6.2};
        const Transform2D T2{{-3.0,6.0},2.3};
        T1*=T2;

        CHECK_THAT(T1.rotation(),WithinRel(2.217, 0.001));
        CHECK_THAT(T1.translation().x, WithinRel(1.5089, 0.0001));
        CHECK_THAT(T1.translation().y,WithinRel(4.2285, 0.0001));
    }

    SECTION("Test 3"){
        Transform2D T1{{2.0,5.5},-6.2};
        const Transform2D T2{{-3.4, 2.333},-8.3};
        T1*=T2;

        CHECK_THAT(T1.rotation(),WithinRel(-1.9336, 0.001));
        CHECK_THAT(T1.translation().x, WithinRel(-1.5821, 0.0001));
        CHECK_THAT(T1.translation().y,WithinRel(7.5424, 0.0001));
    }
}

TEST_CASE("Translation", "[translation]"){
    Transform2D T{{1.0,5.0},PI/4};
    CHECK_THAT(T.translation().x, WithinRel(1.0));
    CHECK_THAT(T.translation().y, WithinRel(5.0));
}

TEST_CASE("Rotation", "[quadrant 1]"){
    Transform2D T{{3.0,5.0},PI/4.0};
    CHECK_THAT(T.rotation(), WithinRel(PI/4.0));
}

TEST_CASE("Rotation", "[quadrant 2]"){
    Transform2D T{{7.0,2.0},3.0*PI/4.0};
    CHECK_THAT(T.rotation(), WithinRel(3*PI/4.0));
}
TEST_CASE("Rotation", "[quadrant 3]"){
    Transform2D T{{-98.0,5.0},5.0*PI/4.0};
    CHECK_THAT(T.rotation(), WithinRel(5.0*PI/4.0) || WithinRel(-3.0*PI/4.0));
}

TEST_CASE("Rotation", "[quadrant 4]"){
    Transform2D T{{1.0,5.0},-PI/4.0};
    CHECK_THAT(T.rotation(), WithinRel(-PI/4.0));
}

TEST_CASE("Rotation", "[theta = 0]"){
    Transform2D T{{1.0,5.0}, 0.0};
    CHECK_THAT(T.rotation(), WithinAbs(0.0, zero_margin));
}

TEST_CASE("Rotation", "[theta = pi]"){
    Transform2D T{{1.0,5.0}, 3*PI};
    CHECK_THAT(T.rotation(), WithinRel(PI));
}


TEST_CASE("Operator - () (Twist) ", "[operator]"){
    const Transform2D T1{{1.0,3.0},0.0};
    const Twist2D Tw{1.0, {3.0, -5.0}};
    const Twist2D Twf = T1(Tw);

    CHECK_THAT(Twf.w, WithinRel(1.0));
    CHECK_THAT(Twf.v.x, WithinRel(6.0));
    CHECK_THAT(Twf.v.y, WithinRel(-6.0));
}

TEST_CASE("Operator - <<", "[operator]"){
    const Transform2D T1{{1.0,3.0}, 0.0};
    std::stringstream os{};
    os << T1;
    std::stringstream ss{};
    ss << "deg: 0 x: 1 y: 3";

    REQUIRE(os.str() == ss.str());
}

TEST_CASE("Operator >>", "[operator]"){

    Transform2D Tf{};
    std::stringstream is{};
    is.str("deg: 0.0 x: 1.0 y: 3.0\n");
    is >> Tf;
    
    
    CHECK_THAT(Tf.rotation(), WithinAbs(0.0, zero_margin));
    CHECK_THAT(Tf.translation().x, WithinRel(1.0));
    CHECK_THAT(Tf.translation().y, WithinRel(3.0));
}



TEMPLATE_TEST_CASE("Integrate twist", "[integrate]", Twist2D, Transform2D){
    Twist2D Vb{};

    SECTION("pure translation"){
        Vb.w = 0.0;
        Vb.v = {2.0,2.0};
        Transform2D Tb_bp = integrate_twist(Vb);
        CHECK_THAT(Tb_bp.translation().x, WithinRel(2.0));
        CHECK_THAT(Tb_bp.translation().y, WithinRel(2.0));
        CHECK_THAT(Tb_bp.rotation(), WithinAbs(0.0, zero_margin));
    }

    SECTION("pure rotation"){
        Vb.w = PI;
        Vb.v = {0.0,0.0};
        Transform2D Tb_bp = integrate_twist(Vb);
        CHECK_THAT(Tb_bp.translation().x, WithinAbs(0.0, zero_margin));
        CHECK_THAT(Tb_bp.translation().y, WithinAbs(0.0, zero_margin));
        CHECK_THAT(Tb_bp.rotation(), WithinRel(PI));
    }
    
    SECTION("simultaneous translation and rotation 1"){
        Vb.w = PI;
        Vb.v = {2.0,2.0};
        Transform2D Tb_bp = integrate_twist(Vb);
        CHECK_THAT(Tb_bp.translation().x, WithinRel(4.0/PI, 0.001));
        CHECK_THAT(Tb_bp.translation().y, WithinRel(-4.0/PI, 0.001));
        CHECK_THAT(Tb_bp.rotation(), WithinRel(PI) || WithinRel(-PI));
    }


    SECTION("simultaneous translation and rotation 2"){
        Vb.w = 5.43;
        Vb.v = {3.56,5.45};
        Transform2D Tb_bp = integrate_twist(Vb);
        CHECK_THAT(Tb_bp.translation().x, WithinRel(-0.1502, .001));
        CHECK_THAT(Tb_bp.translation().y, WithinRel(-0.9806, 0.001));
        CHECK_THAT(Tb_bp.rotation(), WithinRel(0.8532, 0.0001) || WithinRel(-0.8532, 0.0001));
    }
}

TEMPLATE_TEST_CASE("Inverse Kinematics", "[inverse]", Twist2D, Transform2D){
    Twist2D Vb{};
    DiffDrive bot{.16,.033};

    SECTION("pure translation"){
        Vb.w = 0.0;
        Vb.v = {5.0,0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        CHECK_THAT(phidot.l, WithinRel(151.5151, 0.0001));
        CHECK_THAT(phidot.r, WithinRel(151.5151, 0.0001));
    }

    SECTION("pure translation"){
        Vb.w = 0.0;
        Vb.v = {-52.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        CHECK_THAT(phidot.l, WithinRel(-1575.7575, 0.0001));
        CHECK_THAT(phidot.r, WithinRel(-1575.7575, 0.0001));
    }


    SECTION("pure rotation"){
        Vb.w = PI;
        Vb.v = {0.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        CHECK_THAT(phidot.l, WithinRel(-2.4242*PI, 0.0001));
        CHECK_THAT(phidot.r, WithinRel(2.4242*PI, 0.0001));
    }
    
    SECTION("pure rotation"){
        Vb.w = -36.0;
        Vb.v = {0.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        CHECK_THAT(phidot.l, WithinRel(87.2727, 0.0001));
        CHECK_THAT(phidot.r, WithinRel(-87.2727, 0.0001));
    }

    SECTION("simultaneous translation and rotation 1"){
        Vb.w = PI;
        Vb.v = {2.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        CHECK_THAT(phidot.l, WithinRel(52.9901, 0.0001));
        CHECK_THAT(phidot.r, WithinRel(68.222, 0.0001));
    }


    SECTION("simultaneous translation and rotation 2"){
        Vb.w = -5.43;
        Vb.v = {3.56,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        CHECK_THAT(phidot.l, WithinRel(121.0424, 0.0001));
        CHECK_THAT(phidot.r, WithinRel(94.715, 0.0001));
    }
}

TEMPLATE_TEST_CASE("forward Kinematics", "[forward]", Twist2D, Transform2D){
    Twist2D Vb{};
    DiffDrive bot{.16,.033};

    SECTION("pure translation"){
        Vb.w = 0.0;
        Vb.v = {5.0,0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        Vb = bot.forward_kin(phidot.l, phidot.r);
        CHECK_THAT(Vb.w, WithinRel(0.0, 0.0001));
        CHECK_THAT(Vb.v.x, WithinRel(5.0, 0.0001));
        CHECK_THAT(Vb.v.y, WithinRel(0, 0.0001));
    }

    SECTION("one revolution forward"){
        bot.forward_kin(2*PI, 2*PI);
        CHECK_THAT(bot.q.x, WithinAbs(0.2073, 0.0001));
    }

    SECTION("max speed at 200 Hz for 1 timer callback"){
        const auto phidot_l = 265/41.667;
        const auto phidot_r = 265/41.667;
        bot.forward_kin(phidot_l/200, phidot_r/200);
        CHECK_THAT(bot.q.x, WithinAbs(0.00105, 0.0001));
    }

    SECTION("max speed at 200 Hz for 1 second"){
        const auto phidot_l = 265/41.667;
        const auto phidot_r = 265/41.667;
        bot.forward_kin(phidot_l, phidot_r);
        CHECK_THAT(bot.q.x, WithinAbs(0.20987, 0.0001));
    }

    SECTION("17 revolutions forward"){
        bot.forward_kin(17*2*PI, 17*2*PI);
        CHECK_THAT(bot.q.x, WithinAbs(3.5249, 0.0001));
    }

    SECTION("pure translation"){
        Vb.w = 0.0;
        Vb.v = {-52.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        Vb = bot.forward_kin(phidot.l, phidot.r);
        CHECK_THAT(Vb.w, WithinRel(0.0, 0.0001));
        CHECK_THAT(Vb.v.x, WithinRel(-52.0, 0.0001));
        CHECK_THAT(Vb.v.y, WithinRel(0, 0.0001));
    }


    SECTION("pure rotation"){
        Vb.w = PI;
        Vb.v = {0.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        Vb = bot.forward_kin(phidot.l, phidot.r);
        CHECK_THAT(Vb.w, WithinRel(PI, 0.0001));
        CHECK_THAT(Vb.v.x, WithinRel(0.0, 0.0001));
        CHECK_THAT(Vb.v.y, WithinRel(0.0, 0.0001));
    }
    
    SECTION("pure rotation"){
        Vb.w = -36.0;
        Vb.v = {0.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        Vb = bot.forward_kin(phidot.l, phidot.r);
        CHECK_THAT(Vb.w, WithinRel(-36.0, 0.0001));
        CHECK_THAT(Vb.v.x, WithinRel(0.0, 0.0001));
        CHECK_THAT(Vb.v.y, WithinRel(0.0, 0.0001));
    }

    SECTION("simultaneous translation and rotation 1"){
        Vb.w = PI;
        Vb.v = {2.0,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        Vb = bot.forward_kin(phidot.l, phidot.r);
        CHECK_THAT(Vb.w, WithinRel(PI, 0.0001));
        CHECK_THAT(Vb.v.x, WithinRel(2.0, 0.0001));
        CHECK_THAT(Vb.v.y, WithinRel(0.0, 0.0001));
    }


    SECTION("simultaneous translation and rotation 2"){
        Vb.w = -5.43;
        Vb.v = {3.56,0.0};
        Wheel_Vel phidot = bot.inverse_kin(Vb);
        Vb = bot.forward_kin(phidot.l, phidot.r);
        CHECK_THAT(Vb.w, WithinRel(-5.43, 0.0001));
        CHECK_THAT(Vb.v.x, WithinRel(3.56, 0.0001));
        CHECK_THAT(Vb.v.y, WithinRel(0, 0.0001));
    }

    SECTION("Impossible to follow twist"){
        Vb.w = -5.43;
        Vb.v = {3.56,5.0};
        CHECK_THROWS(bot.inverse_kin(Vb));
}
}

TEMPLATE_TEST_CASE("find angle", "[find angle]", double){ 
    double radians{};
    double dx{};
    double dy{};

    SECTION("pi/4"){
        dx = .038*cos(PI/4);
        dy = .038*sin(PI/4);
        radians = turtlelib::find_angle(dx,dy);
        CHECK_THAT(radians, WithinRel(PI/4,.0001));
    }

    SECTION("0"){
        dx = 5.0;
        dy = 0.0;
        radians = turtlelib::find_angle(dx,dy);
        CHECK_THAT(radians, WithinAbs(0.0,.0001));
    }

    SECTION("PI"){
        dx = -5.0;
        dy = 0.0;
        radians = turtlelib::find_angle(dx,dy);
        CHECK_THAT(radians, WithinAbs(PI,.0001));
    }

    SECTION("2nd quad"){
        dx = -1.56;
        dy = 2.434;
        radians = turtlelib::find_angle(dx,dy);
        CHECK_THAT(radians, WithinAbs(2.1407,.0001));
    }

    SECTION("3rd quad"){
        dx = -1.56;
        dy = -2.434;
        radians = turtlelib::find_angle(dx,dy);
        CHECK_THAT(radians, WithinAbs(-2.1407,.0001));
    }

    SECTION("4th quad"){
        dx = 1.56;
        dy = -2.434;
        radians = turtlelib::find_angle(dx,dy);
        CHECK_THAT(radians, WithinAbs(-1.0008,.0001));
    }
}

TEMPLATE_TEST_CASE("collision_detection", "[collision_detection]", Config){
    std::vector<double> obstacles_x = {-0.6, 0.7, 0.5};
    std::vector<double> obstacles_y = {-0.8, -0.7, 0.9};
    double obstacles_r = 0.038;
    double collision_radius = 0.11;
    Config q{0.0,0.0,0.0}; Config q_new{0.0,0.0,0.0};

    SECTION("quadrant 1 (radius away = 0.1)"){
        q.x = -0.575;
        q.y = -0.775;
        q_new = collision_detection(q,collision_radius, obstacles_x, obstacles_y, obstacles_r);
        CHECK_THAT(q_new.x, WithinAbs(-.495,.01));
        CHECK_THAT(q_new.y, WithinAbs(-.695, 0.01));
    }
}

TEMPLATE_TEST_CASE("check_direction", "[check direction]", Config, double){
    double ix{};
    double iy{};
    bool check{};
    double max_x{}; double max_y{};
    Config q{0.0,0.0,0.0};

    SECTION("angle = 0, dy = 0"){
        q.x = 0.0;
        q.y = 0.9;
        ix = 0.462;
        iy = 0.9;
        max_x = 3.5; max_y = 0.9;
        check = check_direction(q, ix, iy, max_x, max_y);
        CHECK(check == true);
    }

    SECTION("angle = -PI, dy = 0"){
        q.x = 0.0;
        q.y = 0.9;
        ix = 0.462;
        iy = 0.9;
        max_x = -3.5; max_y = 0.9;
        check = check_direction(q, ix, iy, max_x, max_y);
        CHECK(check == false);
    }

    SECTION("angle = 0, dx = 0"){
        q.x = 0.5;
        q.y = 0.7;
        ix = 0.5;
        iy = 0.9-.038;
        max_x = 0.5; max_y = 3.5+0.9;
        check = check_direction(q, ix, iy, max_x, max_y);
        CHECK(check == true);
    }
}

TEMPLATE_TEST_CASE("obstacle_range", "[range of obstacles]", double){
    std::vector<double> obstacles_x = {-0.6, 0.7, 0.5};
    std::vector<double> obstacles_y = {-0.8, -0.7, 0.9};
    double obstacles_r = 0.038;
    double range{}; double angle{};
    double range_max = 3.5;
    //double angle_increment = 0.01745329238474369;
    Config q{0.0,0.0,0.0};

    SECTION("angle = 0, dy = 0"){
        q.x = 0.0;
        q.y = 0.9;
        q.theta = 0.0;
        angle = 0.0;
        range = range_obstacles(q, range_max, obstacles_x, obstacles_y, obstacles_r, angle);
        CHECK_THAT(range, WithinAbs(0.462,.01));
    }

    SECTION("angle = 5.1247, dy = 0"){
        q.x = 0.0;
        q.y = 0.9;
        q.theta = 0.0;
        angle = 5.124799;
        range = range_obstacles(q, range_max, obstacles_x, obstacles_y, obstacles_r, angle);
        CHECK_THAT(range, WithinAbs(1.7084, .01));
    }

    SECTION("angle = pi/4, dy = 0"){
        q.x = 0.0;
        q.y = 0.9;
        q.theta = -PI/4;
        angle = PI/4;
        range = range_obstacles(q, range_max, obstacles_x, obstacles_y, obstacles_r, angle);
        CHECK_THAT(range, WithinAbs(0.462,.01));
    }

   SECTION("angle = -pi, dy = 0"){
        q.x = 0.0;
        q.y = 0.9;
        q.theta = 0.0;
        angle = PI;
        range = range_obstacles(q, range_max, obstacles_x, obstacles_y, obstacles_r, angle);
        CHECK_THAT(range, WithinAbs(0.0,.01));
    }

    SECTION("angle = pi/2, dx = 0"){
        q.x = 0.5;
        q.y = 0.7;
        q.theta = 0.0;
        angle = PI/2;
        range = range_obstacles(q, range_max, obstacles_x, obstacles_y, obstacles_r, angle);
        CHECK_THAT(range, WithinAbs(0.9-.038 - 0.7,.01));
    }

    SECTION("angle = -pi/2, dx = 0"){
        q.x = 0.5;
        q.y = 1.1;
        q.theta = 0.0;
        angle = -PI/2;
        range = range_obstacles(q, range_max, obstacles_x, obstacles_y, obstacles_r, angle);
        CHECK_THAT(range, WithinAbs(q.y - (0.9+obstacles_r),.01));
    }

    SECTION("angle = pi/4"){
        q.x = 0.162;
        q.y = 0.6;
        q.theta = 0.0;
        angle = PI/4;
        range = range_obstacles(q, range_max, obstacles_x, obstacles_y, obstacles_r, angle);
        CHECK_THAT(range, WithinAbs(0.424264,.01));
    }
}

TEMPLATE_TEST_CASE("wall_range", "[range of walls]", double){
    double arena_x = 5.0;
    double arena_y = 7.0;
    double range{}; double angle{};
    double range_max = 3.5;
    //double angle_increment = 0.01745329238474369;
    Config q{0.0,0.0,0.0};

    SECTION("angle = 0, dy = 0"){
        q.x = 1.0;
        q.y = 0.9;
        q.theta = 0.0;
        angle = 0.0;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(1.5,.01));
    }

    SECTION("angle = pi/4, dy = 0"){
        q.x = 1.0;
        q.y = 0.9;
        q.theta = -PI/4;
        angle = PI/4;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(1.5,.01));
    }

   SECTION("angle = pi, dy = 0"){
        q.x = -1.0;
        q.y = 0.9;
        q.theta = 0.0;
        angle = PI;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(1.5,.01));
    }

    SECTION("angle = pi/2, dx = 0"){
        q.x = 0.5;
        q.y = 0.7;
        q.theta = 0.0;
        angle = PI/2;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(2.8,.01));
    }

    SECTION("angle = pi/2, dx = 0"){
        q.x = -0.5;
        q.y = 0.7;
        q.theta = 0.0;
        angle = PI/2;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(2.8,.01));
    }

    SECTION("angle = 3pi/2, dx = 0"){
        q.x = 0.5;
        q.y = 0.0;
        q.theta = 0.0;
        angle = 3*PI/2;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(3.5,.01));
    }

    SECTION("angle = pi/4"){
        q.x = 0.73223;
        q.y = 1.2322;
        q.theta = 0.0;
        angle = PI/4;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(2.5,.01));
    }

    SECTION("angle = pi/4 - past two walls"){
        q.x = 1.934;
        q.y = 2.9343;
        q.theta = 0.0;
        angle = PI/4;
        range = range_walls(q, range_max, arena_x, arena_y, angle);
        CHECK_THAT(range, WithinAbs(0.8,.01));
    }
}

TEMPLATE_TEST_CASE("ekf prediction step", "[ekf prediction]", double)
{
    Config q_0{0.0,0.0,0.0};
    std::vector<double> m_0 = {1.0,0.0};
    double dx = m_0.at(0) - q_0.x;
    double dy = m_0.at(1) - q_0.y;
    double r = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    double phi = turtlelib::find_angle(dx,dy);
    double Q{0.01};
    int n = (int) m_0.size() / 2;
    turtlelib::EKF f{q_0, n, Q, 0.01};
    f.initialization(0,r,phi);

    SECTION("all zeros"){
        arma::mat A_c;
        f.prediction(q_0);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0,.01));
        //CHECK_THAT(A_c(0,0), WithinAbs(1.0,.01));

    }

    SECTION("correction"){
        Config q{0.5, 0.0, 0.0};
        f.prediction(q);
        int index = 0;
        double x = 0.5;
        double y = 0.0;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, theta = PI/4"){
        Config q{0.0, 0.0, PI/4};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, theta = -PI/4"){
        Config q{0.0, 0.0, -PI/4};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, x = 0.5, theta = -PI/4"){
        Config q{0.5, 0.0, -PI/4};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, theta = PI"){
        Config q{0.0, 0.0, PI};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, theta = 3PI"){
        Config q{0.0, 0.0, 3*PI};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, theta = 7PI"){
        Config q{0.0, 0.0, 7*PI};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, theta = 6PI"){
        Config q{0.0, 0.0, 6*PI};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, x = 0.5, theta = -PI"){
        Config q{0.5, 0.0, -PI};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, x = 0.5, theta = -3.5"){
        Config q{0.0, 0.0, -9*PI};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("correction, x = 0.5, theta = -5PI"){
        Config q{0.0, 0.0, -5*PI};
        f.prediction(q);
        int index = 0;
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.correction(index,x,y);

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("multiple, x = 0.5, theta = 0->pi/3"){

        int index = 0;

        Config q{0.0, 0.0, 0.0};
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.prediction(q);
        f.correction(index,x,y);
        while(q.theta<-3*PI)
        {
            q.theta += -0.1;
            Transform2D T_wr{{q.x,q.y}, q.theta};
            Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
            Transform2D T_ro = T_wr.inv() * T_wo;
            x = T_ro.translation().x;
            y = T_ro.translation().y;
            
        }

        CHECK_THAT(f.zeta_est(3), WithinAbs(1.0, .01));
        CHECK_THAT(f.zeta_est(4), WithinAbs(0.0, .01));
    }

    SECTION("multiple, x=0->, theta = 0->pi/3"){

        int index = 0;

        Config q{0.6, -0.3, 0.0};
        Transform2D T_wr{{q.x,q.y}, q.theta};
        Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
        Transform2D T_ro = T_wr.inv() * T_wo;
        double x = T_ro.translation().x;
        double y = T_ro.translation().y;
        f.prediction(q);
        f.correction(index,x,y);
        while(q.theta<=20*PI)
        {
            q.theta += 0.001;
            // q.x += 0.01;
            // q.y += 0.02;
            Transform2D T_wr{{q.x,q.y}, q.theta};
            Transform2D T_wo{{m_0.at(0), m_0.at(1)}, 0.0};
            Transform2D T_ro = T_wr.inv() * T_wo;
            x = T_ro.translation().x;
            y = T_ro.translation().y;
            f.prediction(q);
            f.correction(index,x,y);
            
        }

        CHECK_THAT(f.zeta_est(1), WithinAbs(0.6, .01));
        CHECK_THAT(f.zeta_est(2), WithinAbs(-0.3, .01));
    }
    

}

TEST_CASE("fake_sensor position test")
{
    Transform2D T_wr{{0.0, 0.0}, PI/2};;
    Transform2D T_wo{{1.0, 0.0}, 0.0};
    Transform2D T_ro = T_wr.inv() * T_wo;
    Transform2D T_revert{{0.0,0.0}, -T_ro.rotation()};
    T_revert *=T_ro;

    CHECK_THAT(T_revert.translation().x, WithinAbs(1.0, .01));
}