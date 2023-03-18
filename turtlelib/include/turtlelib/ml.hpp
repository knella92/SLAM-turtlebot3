#ifndef ML_INCLUDE_GUARD_HPP
#define ML_INCLUDE_GUARD_HPP
/// \file   ml.hpp
/// \brief ML package

#include "ekf.hpp"
#include <vector>
#include <armadillo>

namespace turtlelib
{
    /// Struct for each range datum
    struct RangeID
    {
        /// \brief detected range (meters)
        double range;
        
        /// \brief angle that range was detected at (radians)
        double angle;

        /// \brief cluster ID
        int cluster;

        /// \brief constructor for RangeID
        RangeID();

        /// \brief constructor for RangeID with input values 
        /// \param range detected range (meters)
        /// \param angle angle that range was detected at (radians)
        RangeID(double range, double angle);

    };

    /// \brief Struct consisting of vector of RangeIDs and the number of clusters
    struct Clusters
    {
        /// \brief vector of RangeIDs
        std::vector<RangeID> ranges;

        /// \brief number of clusters
        int n_clusters;

    };

    /// \brief Struct of cluster point coordinates and centroid coordinates
    struct ClustersCentroids
    {
        /// \brief 3D vector of x and y coordinates of cluster points
        std::vector<std::vector<arma::vec>> points;

        /// \brief 2D vector of centroid coordinates
        std::vector<Vector2D> centroids;
    };

    /// \brief Function that clusters data into groups determined by a distance threshold
    /// \param range_data Detected raw range data
    /// \param angle_increment Angle increment between range data points
    /// \param dist_threshold - threshold that determines whether points are considered a cluster
    /// \return Clusters struct of clustered data points
    Clusters clustering(std::vector<double> range_data, double angle_increment, double dist_threshold);
    
    /// \brief - Function that determines centroids of clustered data points
    /// \param cluster clustered data points
    /// \return 2D vector of centroids in order of cluster iD
    std::vector<Vector2D> centroid_finder(Clusters cluster);

    /// \brief Function that drops clusters with less than 4 data points and shifts them to centroids of zero
    /// \param cluster clustered data points
    /// \param centroids centroids of clusters
    /// \return - shifted clustered data points after clusters have been dropped, as well as centroids of those clusters
    ClustersCentroids shift_points(Clusters cluster, std::vector<Vector2D> centroids);

    /// \brief Function that detects circles from clustered points and determine their coordinates and radii
    /// \param clusters clustered data points
    /// \return - vector of detected circle coordinates and radii
    std::vector<Circle> circle_detection(ClustersCentroids clusters);

    /// \brief Function that classifies detected circles as a circle or not. Based on lower and upper threshold of radii
    /// \param detected_circles - detected circle coordinates and radii
    /// \return - vector with true/false values in order of cluster ID
    std::vector<bool> classification(std::vector<Circle> detected_circles);




}


#endif