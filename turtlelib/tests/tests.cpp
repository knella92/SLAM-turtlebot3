#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/rigid2d.hpp"
#include <sstream>
#include <string>



TEMPLATE_TEST_CASE("Normalize rotation angle", "[normalize]", double){
    double rad1{};
    double rad2{};

    SECTION("rad = PI"){
        rad1 = turtlelib::PI;
        rad2 = turtlelib::normalize_angle(rad1);
        REQUIRE(rad2 == turtlelib::PI);
    }

    SECTION("rad = -PI"){
        rad1 = -1.0*turtlelib::PI;
        rad2 = turtlelib::normalize_angle(rad1);
        REQUIRE(rad2 == turtlelib::PI);
    }
    
    SECTION("rad = 0"){
        rad1 = 0.0;
        rad2 = turtlelib::normalize_angle(rad1);
        REQUIRE(rad2 == 0.0);
    }

    SECTION("rad = -PI/4"){
        rad1 = -1.0/4.0 * turtlelib::PI;
        rad2 = turtlelib::normalize_angle(rad1);
        REQUIRE(rad2 == -1.0/4.0 * turtlelib::PI);
    }

    SECTION("rad = 3*PI/2"){
        rad1 = 3.0/2.0 * turtlelib::PI;
        rad2 = turtlelib::normalize_angle(rad1);
        REQUIRE(rad2 == -1.0/2.0 * turtlelib::PI);
    }

    SECTION("rad = -5*PI/2"){
        rad1 = -5.0/2.0 * turtlelib::PI;
        rad2 = turtlelib::normalize_angle(rad1);
        REQUIRE(rad2 == -1.0/2.0 * turtlelib::PI);
    }
}




TEST_CASE("An identity matrix is constructed", "[transform]"){
    turtlelib::Transform2D T{};
    REQUIRE(T.rotation() == 0);
    REQUIRE(T.translation().x == 0);
    REQUIRE(T.translation().y == 0);
}

TEST_CASE("Constructor - Pure Translation", "[transform]"){
    turtlelib::Vector2D vec{25, -19};
    turtlelib::Transform2D T{vec};
    REQUIRE(T.rotation() == 0);
    REQUIRE(T.translation().x == 25);
    REQUIRE(T.translation().y == -19);
}


TEST_CASE("Constructor - Pure Rotation", "[transform]"){
    double radians{turtlelib::PI};
    turtlelib::Transform2D T{radians};
    REQUIRE_THAT(T.rotation(),Catch::Matchers::WithinAbs(-3.14, 0.01)
        || Catch::Matchers::WithinAbs(3.14,0.01));
    REQUIRE(T.translation().x == 0);
    REQUIRE(T.translation().y == 0);
}

TEST_CASE("Constructor - Transform", "[transform]"){
    double radians{turtlelib::PI};
    turtlelib::Vector2D vec{18,-1};
    turtlelib::Transform2D T{vec,radians};
    REQUIRE_THAT(T.rotation(),Catch::Matchers::WithinAbs(-3.14, 0.01)
        || Catch::Matchers::WithinAbs(3.14,0.01));
    REQUIRE(T.translation().x == 18);
    REQUIRE(T.translation().y == -1);
}

TEST_CASE("Operator - () ", "[operator]"){
    double radians{turtlelib::PI};
    turtlelib::Vector2D tvec{18,-1};
    turtlelib::Transform2D T{tvec,radians};
    turtlelib::Vector2D vec{1,1};
    turtlelib::Vector2D test_vec{T(vec)};

    REQUIRE(test_vec.x == 17);
    REQUIRE(test_vec.y == -2);
}

TEST_CASE(" Inverse transform ", "[transform]"){
    double radians{turtlelib::PI*2};
    turtlelib::Vector2D tvec{1,1};
    turtlelib::Transform2D T{tvec,radians};

    REQUIRE_THAT(T.inv().rotation(),Catch::Matchers::WithinAbs(0.0, 0.01)
        || Catch::Matchers::WithinAbs(2*turtlelib::PI,0.01));
    REQUIRE_THAT(T.inv().translation().x,Catch::Matchers::WithinAbs(-1.0, 0.0001));
    REQUIRE_THAT(T.inv().translation().x,Catch::Matchers::WithinAbs(-1.0, 0.0001));
}

TEST_CASE("Operator - *= ", "[operator]"){
    turtlelib::Transform2D T1{{1.0,3.0},0.0};
    turtlelib::Transform2D T2{{-1.0,5.0},turtlelib::PI};
    turtlelib::Transform2D Tf{T1*=T2};

    REQUIRE_THAT(Tf.rotation(),Catch::Matchers::WithinAbs(turtlelib::PI, 0.01)
        || Catch::Matchers::WithinAbs(-1*turtlelib::PI,0.01));
    REQUIRE_THAT(Tf.translation().x,Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(Tf.translation().y,Catch::Matchers::WithinAbs(8.0, 0.0001));
}

TEST_CASE("Translation", "[translation]"){
    turtlelib::Transform2D T{{1.0,5.0},turtlelib::PI/4};
    REQUIRE(T.translation().x == 1.0);
    REQUIRE(T.translation().y == 5.0);
}

TEST_CASE("Rotation", "[rotation]"){
    turtlelib::Transform2D T{{1,5},turtlelib::PI/4};
    REQUIRE_THAT(T.rotation(),Catch::Matchers::WithinAbs(turtlelib::PI/4, 0.01));
}


TEST_CASE("Operator - () (Twist) ", "[operator]"){
    turtlelib::Transform2D T1{{1.0,3.0},0.0};
    turtlelib::Twist2D Tw{1.0, {3.0, -5.0}};
    turtlelib::Twist2D Twf = T1(Tw);

    REQUIRE(Twf.w == 1.0);
    REQUIRE(Twf.v.x == 6.0);
    REQUIRE(Twf.v.y == -6.0);
}

TEST_CASE("Operator - <<", "[operator]"){
    turtlelib::Transform2D T1{{1.0,3.0}, 0.0};
    std::stringstream os{};
    os << T1;
    std::stringstream ss{};
    ss << "deg: 0 x: 1 y: 3";

    REQUIRE(os.str() == ss.str());
}

TEST_CASE("Operator >>", "[operator]"){

    turtlelib::Transform2D Tf{};
    std::stringstream is{};
    is.str("deg: 0.0 x: 1.0 y: 3.0\n");
    is >> Tf;
    
    
    REQUIRE(Tf.rotation() == 0.0);
    REQUIRE(Tf.translation().x == 1.0);
    REQUIRE(Tf.translation().y == 3.0);
}