#ifndef ML_INCLUDE_GUARD_HPP
#define ML_INCLUDE_GUARD_HPP
/// \file   ml.hpp
/// \brief ML package

#include "ekf.hpp"
#include <vector>

namespace turtlelib
{

    struct Point
    {

        double x, y;

        int cluster;

        double minDist;

        Point();

        Point(double x, double y);

        double sq_distance(Point p);

    };

    void kMeansClustering(std::vector<Point> * points, int epochs, int k);


}


#endif