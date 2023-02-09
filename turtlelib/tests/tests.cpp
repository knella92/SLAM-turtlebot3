#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include <sstream>
#include <string>
#include <cmath>

using turtlelib::PI;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::Vector2D;
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

    SECTION("rad = -PI"){
        rad1 = -1.0*PI;
        rad2 = turtlelib::normalize_angle(rad1);
        CHECK_THAT(rad2, WithinRel(PI));
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

    CHECK_THAT(T.inv().rotation(), WithinRel(1.0));
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

        CHECK_THAT(T1.rotation(),WithinRel(3*PI/4.0)||WithinRel(-PI/2.0));
        CHECK_THAT(T1.translation().x, WithinRel(-1.0-sqrt(2)));
        CHECK_THAT(T1.translation().y,WithinRel(2+4*sqrt(2)));
    }

    SECTION("Test 2"){
        Transform2D T1{{4.0,-2.0},6.2};
        const Transform2D T2{{-3.0,6.0},2.3};
        T1*=T2;

        CHECK_THAT(T1.rotation(),WithinRel(2.217, 0.001)||WithinRel(-0.8412, 0.001));
        CHECK_THAT(T1.translation().x, WithinRel(1.5089, 0.0001));
        CHECK_THAT(T1.translation().y,WithinRel(4.2285, 0.0001));
    }

    SECTION("Test 3"){
        Transform2D T1{{2.0,5.5},-6.2};
        const Transform2D T2{{-3.4, 2.333},-8.3};
        T1*=T2;

        CHECK_THAT(T1.rotation(),WithinRel(1.9336, 0.001)||WithinRel(-1.1247, 0.001));
        CHECK_THAT(T1.translation().x, WithinRel(-1.5821, 0.0001));
        CHECK_THAT(T1.translation().y,WithinRel(7.5424, 0.0001));
    }
}

TEST_CASE("Translation", "[translation]"){
    Transform2D T{{1.0,5.0},PI/4};
    CHECK_THAT(T.translation().x, WithinRel(1.0));
    CHECK_THAT(T.translation().y, WithinRel(5.0));
}

TEST_CASE("Rotation", "[rotation]"){
    Transform2D T{{1.0,5.0},PI/4.0};
    CHECK_THAT(T.rotation(), WithinRel(PI/4.0));
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