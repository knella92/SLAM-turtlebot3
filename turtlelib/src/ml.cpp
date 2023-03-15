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
        int k{0};
        int k_b{0};
        bool check_end{false};
        Clusters lidar{};
        RangeID range_i{};

        if(range_data.at(0) != 0.0)
        {
            range_i.range = {range_data.at(0)};
            range_i.cluster = 0;
            k++;
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
                    // dropper //
                    if(i < 3)
                    {
                        check_end = true;
                        k_b = k;
                        cluster_id++;
                        range_i.cluster = cluster_id;
                    }
                    else if(k<3)
                    {
                        while(k>0)
                        {
                            lidar.ranges.at(i- k).cluster = -1;
                            k--;
                        }
                        range_i.cluster = -1;
                    }
                    else{
                        cluster_id++;
                        range_i.cluster = cluster_id;
                    }
                    k = 0;
                    // dropper //
                }
                else
                {
                    k++;
                    range_i.cluster = cluster_id;
                }
            }
            // std::cout << cluster_id << std::endl;
            lidar.ranges.push_back(range_i);
        }
    
        //last batch
        for(int i{n/20*17}; i < n; i++)
        {
            range_i = RangeID();
            if (!almost_equal(range_data.at(i), 0.0, 0.1))
            {
                range_i.range = range_data.at(i);
                range_i.angle = i*angle_increment;
                if(std::abs(range_i.range - lidar.ranges.at(i-1).range) > dist_threshold)
                {
                    if(std::abs(range_i.range - lidar.ranges.at(0).range) < dist_threshold)
                    {
                        k_b++;
                        range_i.cluster = 0;
                    }
                    else if(k<3)
                    {
                        while(k>0)
                        {
                            lidar.ranges.at(i- k).cluster = -1;
                            k--;
                        }
                        range_i.cluster = -1;
                    }
                    else{
                        cluster_id++;
                        range_i.cluster = cluster_id; 
                    }
                    k = 0;
                }
                else
                {
                    k++;
                    range_i.cluster = cluster_id;
                }
            } 
            lidar.ranges.push_back(range_i);
        }

        // check if cluster 0
        if(check_end == true && k_b<3)
        {
            if(lidar.ranges.at(n-2).cluster == 0)
            {
                lidar.ranges.at(n-2).cluster = -1;
            }
            if(lidar.ranges.at(n-1).cluster == 0)
            {
                lidar.ranges.at(n-1).cluster = -1;
            }
            if(lidar.ranges.at(0).cluster == 0)
            {
                lidar.ranges.at(0).cluster = -1;
            }
            if(lidar.ranges.at(1).cluster == 0)
            {
                lidar.ranges.at(1).cluster = -1;
            }
            cluster_id--;
        }
        lidar.max_cluster = cluster_id;
        return lidar;
    }

    


    Centroids centroid_finder(Clusters cluster)
    {
        arma::vec x_i(cluster.max_cluster+1);
        arma::vec y_i(cluster.max_cluster+1);
        double counter{0.0};
        for(int j{0}; j < (cluster.max_cluster + 1); j++)
        {
            counter = 0.0;
            for(int i{0}; i < (int) cluster.ranges.size(); i++)
            {
                if(cluster.ranges.at(i).cluster == j)
                {
                    x_i(j) += cluster.ranges.at(i).range * cos(cluster.ranges.at(i).angle);
                    y_i(j) += cluster.ranges.at(i).range * sin(cluster.ranges.at(i).angle);
                    counter +=1.0;
                }
            }
            if (counter != 0.0)
            {    
                x_i(j) /= counter;
                y_i(j) /= counter;
            }
        }
        return {x_i, y_i};
    }




    arma::mat HAF_finder(Clusters cluster)
    {
        Centroids centroid_vec = centroid_finder(cluster);
        arma::vec z(arma::size(centroid_vec.x_i));
        z = centroid_vec.x_i % centroid_vec.x_i + centroid_vec.y_i % centroid_vec.y_i;
        double z_bar = mean(z);
        arma::mat Z(z.n_elem, 4);
        for (int i{0}; i < (int) z.n_elem; i++)
        {
            Z(i,0) = z(i);
            Z(i,1) = centroid_vec.x_i(i);
            Z(i,2) = centroid_vec.y_i(i);
            Z(i,3) = 1.0;
        }
        arma::mat M = 1.0/((double) z.n_elem) * Z.t()*Z;
        arma::mat Hinv = {{0.0, 0.0, 0.0, 0.5},
                          {0.0, 1.0, 0.0, 0.0},
                          {0.0, 0.0, 1.0, 0.0},
                          {0.5, 0.0, 0.0, -2*z_bar}};
        return Hinv;
    }











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