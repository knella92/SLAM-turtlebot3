#include "turtlelib/ekf.cpp"
#include <cmath>
#include <iostream>
#include <cfloat>

namespace turtlelib
{

    Point::Point()
     : x{0.0}, y{0.0}, cluster{-1}, minDist(__DBL_MAX__)
    {}

    Point::Point(double x, double y)
     : x{x}, y{y}, cluster{-1}, minDist{__DBL_MAX__}
     {}

    double Point::sq_distance(Point p)
    {
        return (std::pow(p.x-x, 2.0) + std::pow(p.y - y, 2.0));
    }

    kMeansClustering(std::vector<Point> * points, int epochs, int k)
    {
        int n = vector.size();
        std::vector<Point> centroids{};
        for(int i{0}; i < k; i++)
        {
            centroids.push_back(points->at(get_random() % n))
        }
        
    }

    
}