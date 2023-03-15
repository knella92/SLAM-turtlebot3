#include "turtlelib/ml.hpp"
#include <cmath>
#include <iostream>
#include <cfloat>

namespace turtlelib
{

    RangeID::RangeID()
     : range{0.0}, angle{0.0}, cluster{-1}
    {}

    RangeID::RangeID(double range, double angle)
     : range{range}, angle{angle}, cluster{-1}
     {}

    Clusters clustering(std::vector<double> range_data, double angle_increment, double dist_threshold)
    {
        
        int n = range_data.size();
        int cluster_id{0};
        Clusters lidar{};
        RangeID range_i{};
        if(range_data.at(0) != 0.0)
        {
            range_i = {range_data.at(0), 0.0};
        }
        lidar.ranges.push_back(range_i);

        for(int i{1}; i < n/20*17; i++)
        {
            range_i = RangeID();
            if (!almost_equal(range_data.at(i), 0.0, 0.1))
            {
                range_i.range = range_data.at(i);
                range_i.angle = i*angle_increment;
                if(std::abs(range_i.range - lidar.ranges.at(i-1).range) > dist_threshold)
                {
                    cluster_id++;
                }
                range_i.cluster = cluster_id;
            }
            // std::cout << cluster_id << std::endl;
            lidar.ranges.push_back(range_i);
        }
        lidar.max_cluster = cluster_id;
        for(int i{n/20*17}; i < n; i++)
        {
            range_i = RangeID();
            if (!almost_equal(range_data.at(i), 0.0, 0.1))
            {
                range_i.range = range_data.at(i);
                // std::cout<<ranges.at(i) << std::endl;
                range_i.angle = i*angle_increment;
                if(std::abs(range_i.range - lidar.ranges.at(i-1).range) > .05)
                {
                    if(std::abs(range_i.range - lidar.ranges.at(0).range) < dist_threshold)
                    {
                        cluster_id = 0;
                    }
                    else{
                        cluster_id++;
                        lidar.max_cluster = cluster_id;
                    }
                }
                range_i.cluster = cluster_id;
                // std::cout << range_i.cluster << std::endl;
            } 
            lidar.ranges.push_back(range_i);
        }
        return lidar;
    }

    void drop_clusters(Clusters & cluster)
    {
        int k{0};
        int n = cluster.ranges.size();
        for (int i{0}; i < (cluster.max_cluster + 1); i++)
        {
            for(int range_index{0}; range_index < n; range_index++)
            {
                if(cluster.ranges.at(range_index).cluster == i)
                {
                    k++;
                }
                else
                {
                    if(k<3)
                    {
                        while(k>0)
                        {
                            cluster.ranges.at(range_index - k).cluster = -1;
                            k--;
                        }
                    }
                    k = 0;
                }
            }
        }
    }


    Centroids centroid_finder(Clusters cluster)
    {
        arma::vec x_i((int) cluster.ranges.size());
        arma::vec y_i((int) cluster.ranges.size());
        double counter{0.0};
        for(int j{0}; j < cluster.max_cluster + 1; j++)
        {
            for(int i{0}; i < (int) cluster.ranges.size(); i++)
            {
                if(cluster.ranges.at(i).cluster == i)
                {
                    x_i(j) += cluster.ranges.at(i).range * cos(cluster.ranges.at(i).angle);
                    y_i(j) += cluster.ranges.at(i).range * sin(cluster.ranges.at(i).angle);
                    counter +=1.0;
                }
            }
            x_i(j) /= counter;
            y_i(j) /= counter;
        }
        return {x_i, y_i};
    }

    arma::mat H_finder(Cluster clusters)
    {
        Centroids centroid_vec = centroid_finder(clusters);
        arma::vec z(size(centroid_vec.x_i));
        z = centroid_vec.x_i % centroid_vec.x_i + centroid_vec.y_i % centroid_vec.y_i;
        double z_bar = mean(z);
        arma::mat Z(size(z), 4);
        for (int i{0}; i < size(z); i++)
        {
            Z(i,0) = z(i);
            Z(i,1) = centroid_vec.x_i(i);
            Z(i,2) = centroid_vec.y_i(i);
            Z(i,3) = 1.0;
        }
        arma::mat M = 1/size(z) * Z.t()*Z;
        arma::mat Hinv(4,4) = {{0.0, 0.0, 0.0, 0.5},
                               {0.0, 1.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0},
                               {0.5, 0.0, 0.0, -2*z_bar}};
        return Hinv;
    }

















}