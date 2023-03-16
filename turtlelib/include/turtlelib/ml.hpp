#ifndef ML_INCLUDE_GUARD_HPP
#define ML_INCLUDE_GUARD_HPP
/// \file   ml.hpp
/// \brief ML package

#include "ekf.hpp"
#include <vector>

namespace turtlelib
{

    struct RangeID
    {

        double range, angle;

        int cluster;

        RangeID();

        RangeID(double range, double angle);

    };

    struct Clusters
    {
        std::vector<RangeID> ranges;

        int n_clusters;

    };

    struct Circle
    {
        double a;
        double b;
        double R;
    };

    Clusters clustering(std::vector<double> range_data, double angle_increment, double dist_threshold);
    
    // void drop_clusters(Clusters & cluster);

    std::vector<Vector2D> centroid_finder(Clusters cluster);

    std::vector<std::vector<arma::vec>> shift_points(Clusters cluster);

    // arma::mat data_matrix

    std::vector<Circle> circle_detection(std::vector<std::vector<arma::vec>> cluster_pts);


}


#endif