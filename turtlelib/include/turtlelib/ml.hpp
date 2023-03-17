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

    struct ClustersCentroids
    {
        std::vector<std::vector<arma::vec>> points;
        std::vector<Vector2D> centroids;
    };

    Clusters clustering(std::vector<double> range_data, double angle_increment, double dist_threshold);
    
    // void drop_clusters(Clusters & cluster);

    std::vector<Vector2D> centroid_finder(Clusters cluster);

    ClustersCentroids shift_points(Clusters cluster, std::vector<Vector2D> centroids);

    // arma::mat data_matrix

    std::vector<Circle> circle_detection(ClustersCentroids clusters);

    std::vector<bool> classification(std::vector<Circle> detected_circles);


}


#endif