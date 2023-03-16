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
            range_i.range = {range_data.at(0)};
            range_i.cluster = 0;
        }
        lidar.ranges.push_back(range_i);

        for(int i{1}; i < n-5; i++)
        {
            range_i = RangeID{};
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
            lidar.ranges.push_back(range_i);
        }
    
        //last batch
        for(int i{n-5}; i < n; i++)
        {
            range_i = RangeID{};
            if (!almost_equal(range_data.at(i), 0.0, 0.1))
            {
                range_i.range = range_data.at(i);
                range_i.angle = i*angle_increment;
                if(std::abs(range_i.range - lidar.ranges.at(i-1).range) > dist_threshold)
                {
                    if(std::abs(range_i.range - lidar.ranges.at(0).range) < dist_threshold)
                    {
                        range_i.cluster = 0;
                    }
                    else
                    {
                        cluster_id++;
                        range_i.cluster = cluster_id; 
                    }
                }
                else
                {
                    range_i.cluster = cluster_id;
                }
            } 
            lidar.ranges.push_back(range_i);
        }

        lidar.n_clusters = cluster_id+1;
        return lidar;
    }

    


    std::vector<Vector2D> centroid_finder(Clusters cluster)
    {
        std::vector<Vector2D> centroids{};
        double elements{0.0};
        // int n_c{0};
        for(int j{0}; j < (cluster.n_clusters); j++)
        {
            Vector2D center{};
            elements = 0.0;
            // x(index) = 0.0;
            // y(index) = 0.0;
            for(int i{0}; i < (int) cluster.ranges.size(); i++)
            {
                if(cluster.ranges.at(i).cluster == j)
                {
                    center.x += cluster.ranges.at(i).range * cos(cluster.ranges.at(i).angle);
                    center.y += cluster.ranges.at(i).range * sin(cluster.ranges.at(i).angle);
                    elements +=1.0;
                }
            }
            if (elements > 2.0)
            {    
                center.x /= elements;
                center.y /= elements; 
                // n_c++;
            }
            else{
                center.x = 0.0;
                center.y = 0.0;
            }
            centroids.push_back(center);
        }
        // x_i.resize(index);
        // y_i.resize(index);
        // n_clusters = n_c;

        return centroids;
    }




    std::vector<std::vector<arma::vec>> shift_points(Clusters cluster)
    {
        std::vector<Vector2D> centroids = centroid_finder(cluster);
        std::vector<std::vector<arma::vec>> cluster_pts(cluster.n_clusters);

        for (int i{0}; i < cluster.n_clusters; i++)
        {
            // 2 vectors (x and y) representing x and y coordinates of points in a cluster
            arma::vec x(100);
            arma::vec y(100);
            cluster_pts.at(i).push_back(x);
            cluster_pts.at(i).push_back(y);
        }
        
        std::vector<int> point_index(cluster.n_clusters);
        for(int i{0}; i < cluster.n_clusters; i++)
        {
            point_index.at(i) = 0;
        }

        for(int i{0}; i < (int) cluster.ranges.size(); i++)
        {
            int cluster_index = cluster.ranges.at(i).cluster;
            if(cluster_index != -1)
            {
                if(centroids.at(cluster_index).x == 0.0 && centroids.at(cluster_index).y == 0.0)
                {
                    continue;
                }
                int p_i = point_index.at(cluster_index);
                cluster_pts.at(cluster_index).at(0)(p_i) = cluster.ranges.at(i).range * cos(cluster.ranges.at(i).angle) - centroids.at(cluster_index).x;

                cluster_pts.at(cluster_index).at(1)(p_i) = cluster.ranges.at(i).range * sin(cluster.ranges.at(i).angle) - centroids.at(cluster_index).y;

                point_index.at(cluster_index)++;
            }
        }

        // resize arma vectors
        for(int i{0}; i < (int) point_index.size(); i++)
        {
            cluster_pts.at(i).at(0).resize(point_index.at(i));
            cluster_pts.at(i).at(1).resize(point_index.at(i));
        }

        // drop clusters with < 2 points
        std::vector<std::vector<arma::vec>> cluster_pts_dropped;
        int index {0};
        for(int i{0}; i < cluster.n_clusters; i++)
        {
            if(centroids.at(i).x == 0.0 && centroids.at(i).y == 0.0){
                continue;
            }
            else
            {
                cluster_pts_dropped.push_back(cluster_pts.at(i));
                index++;
            }
        }

        return cluster_pts_dropped;
    }

    void circle_detection(std::vector<std::vector<arma::vec>> cluster_pts)
    {
        // comput z_i
        std::vector<arma::vec> z((int) cluster_pts.size());
        for(int i{0}; i < (int) cluster_pts.size(); i++)
        {
            arma::vec x_i = cluster_pts.at(i).at(0);
            arma::vec y_i = cluster_pts.at(i).at(1);
            arma::vec z_i = x_i%x_i + y_i%y_i;
            z.push_back(z_i);
        }

    }




        // x_i.resize(vector_index);
        // y_i.resize(vector_index);

        // arma::vec z_i(arma::size(x_i));
        // z_i = x_i % x_i + y_i % y_i;
        // double z_bar = mean(z_i);
        // arma::mat Z(z.n_elem, 4);
        // for (int i{0}; i < (int) z.n_elem; i++)
        // {
        //     Z(i,0) = z(i);
        //     Z(i,1) = centroid_vec.x(i);
        //     Z(i,2) = centroid_vec.y(i);
        //     Z(i,3) = 1.0;
        // }
        // arma::mat M = 1.0/((double) z.n_elem) * Z.t()*Z;
        // arma::mat Hinv = {{0.0, 0.0, 0.0, 0.5},
        //                   {0.0, 1.0, 0.0, 0.0},
        //                   {0.0, 0.0, 1.0, 0.0},
        //                   {0.5, 0.0, 0.0, -2*z_bar}};




// void drop_clusters(Clusters & cluster)
//     {
//         int k{};
//         int n = cluster.ranges.size();
//         bool check_end{false};
//         for (int i{0}; i < (cluster.max_cluster + 1); i++)
//         {
//             k = 0;
//             check_end = false;
//             for(int range_index{0}; range_index < n; range_index++)
//             {
//                 if(cluster.ranges.at(range_index).cluster == i)
//                 {
//                     k++;
//                 }
//                 else if(k>0)
//                 {
//                     if(i < 3)
//                     {
//                         check_end = true;
//                         continue;
//                     }
//                     else if(check_end == true && i == n-1)
//                     {
//                         if(k<3)
//                         {
//                             if(cluster.ranges.at(n-2).cluster == 0)
//                             {
//                                 cluster.ranges.at(n-2).cluster = -1;
//                             }
//                             if(cluster.ranges.at(n-1).cluster == 0)
//                             {
//                                 cluster.ranges.at(n-1).cluster = -1;
//                             }
//                             if(cluster.ranges.at(0).cluster == 0)
//                             {
//                                 cluster.ranges.at(0).cluster = -1;
//                             }
//                             if(cluster.ranges.at(1).cluster == 0)
//                             {
//                                 cluster.ranges.at(1).cluster = -1;
//                             }
//                             cluster.max_cluster--;
//                             break;
//                         }

//                     }
//                     else if(check_end == true)
//                     {
//                         continue;
//                     }
//                     if(k<3)
//                     {
//                         while(k>0)
//                         {
//                             cluster.ranges.at(range_index - k).cluster = -1;
//                             k--;
//                         }
//                         cluster.max_cluster--;
//                         break;
//                     }
//                 }
//             }
//         }
//     }


}