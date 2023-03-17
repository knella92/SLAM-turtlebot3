#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_template_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "turtlelib/ml.hpp"
#include <sstream>
#include <string>
#include <cmath>
#include <armadillo>

using namespace turtlelib;
using Catch::Matchers::WithinAbs;

TEMPLATE_TEST_CASE("circle tests", "[centers and radii]", double){

    SECTION("test 1"){
        RangeID range{};
        Clusters cluster{};
        std::vector<double> input_x = {1.0, 2.0, 5.0, 7.0, 9.0, 3.0};
        std::vector<double> input_y = {7.0, 6.0, 8.0, 7.0, 5.0, 7.0};
        for (int i{0}; i < (int) input_x.size(); i++)
        {
            range = RangeID();
            double r = sqrt(input_x.at(i)*input_x.at(i) + input_y.at(i)*input_y.at(i));
            double angle = atan2(input_y.at(i), input_x.at(i));
            range.range = r;
            range.angle = angle;
            range.cluster = 0;
            cluster.ranges.push_back(range);
        }
        cluster.n_clusters = 1;
        std::vector<Vector2D> centroids = centroid_finder(cluster);
        ClustersCentroids cluster_shift = shift_points(cluster, centroids);
        std::vector<Circle> detected_circles = circle_detection(cluster_shift);
        
        CHECK_THAT(detected_circles.at(0).a, WithinAbs(4.615482, .0001));
        CHECK_THAT(detected_circles.at(0).b, WithinAbs(2.807354, .0001));
        CHECK_THAT(detected_circles.at(0).R, WithinAbs(4.8275, .0001));
        
    }

        SECTION("test 2"){
        RangeID range{};
        Clusters cluster{};
        std::vector<double> input_x = {-1.0, -0.3, 0.3, 1.0};
        std::vector<double> input_y = {0.0, -0.06, 0.1, 0.0};
        for (int i{0}; i < (int) input_x.size(); i++)
        {
            range = RangeID();
            double r = sqrt(input_x.at(i)*input_x.at(i) + input_y.at(i)*input_y.at(i));
            double angle = atan2(input_y.at(i), input_x.at(i));
            range.range = r;
            range.angle = angle;
            range.cluster = 0;
            cluster.ranges.push_back(range);
        }
        cluster.n_clusters = 1;
        std::vector<Vector2D> centroids = centroid_finder(cluster);
        ClustersCentroids cluster_shift = shift_points(cluster, centroids);
        std::vector<Circle> detected_circles = circle_detection(cluster_shift);
        
        CHECK_THAT(detected_circles.at(0).a, WithinAbs(0.4908357, .0001));
        CHECK_THAT(detected_circles.at(0).b, WithinAbs( -22.15212, .0001));
        CHECK_THAT(detected_circles.at(0).R, WithinAbs(22.17979, .0001));
        
    }
}