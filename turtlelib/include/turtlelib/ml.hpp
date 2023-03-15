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

        int max_cluster;

    };

    struct Centroids
    {
        arma::vec x_i;
        arma::vec y_i;
    };

    Clusters clustering(std::vector<double> range_data, double angle_increment, double dist_threshold);
    
    void drop_clusters(Clusters & cluster);

    Centroids centroid_finder(Clusters cluster);

    arma::mat HAF_finder(Clusters cluster);




}


#endif