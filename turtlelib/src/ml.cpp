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
        int cluster_id{-1};
        Clusters lidar{};
        RangeID range_i{};

        if(!almost_equal(range_data.at(0),0.0, 0.1))
        {
            range_i.range = {range_data.at(0)};
            range_i.cluster = 0;
        }
        lidar.ranges.push_back(range_i);

        for(int i{1}; i < n; i++)
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
        if(std::abs(lidar.ranges.at(n-1).range - lidar.ranges.at(0).range) < dist_threshold && lidar.ranges.at(n-1).cluster != -1)
        {
            for(int i{1}; i < n; i++)
            {
                if(lidar.ranges.at(i).cluster == lidar.ranges.at(n-1).cluster)
                {
                    lidar.ranges.at(i).cluster = 0;
                }
                
            }
            if(cluster_id > 0)
            {
                cluster_id--;
            }
        }
        // for(int i{n-1}; i > (n-6) ; i--)
        // {
        //     range_i = RangeID{};
        //     // if (!almost_equal(range_data.at(i), 0.0, 0.1))
        //     // {
        //     //     range_i.range = range_data.at(i);
        //     //     range_i.angle = i*angle_increment;
        //     //     if(std::abs(range_i.range - lidar.ranges.at(i-1).range) > dist_threshold)
        //     //     {
        //     //         if(std::abs(range_i.range - lidar.ranges.at(0).range) < dist_threshold)
        //     //         {
        //     //             range_i.cluster = 0;
        //     //         }
        //     //         else
        //     //         {
        //     //             cluster_id++;
        //     //             range_i.cluster = cluster_id; 
        //     //         }
        //     //     }
        //     //     else
        //     //     {
        //     //         range_i.cluster = cluster_id;
        //     //     }
        //     // } 
        //     // lidar.ranges.push_back(range_i);
        // }

        lidar.n_clusters = cluster_id+1;
        return lidar;
    }

    


    std::vector<Vector2D> centroid_finder(Clusters cluster)
    {
        std::vector<Vector2D> centroids{};
        double elements{0.0};

        for(int j{0}; j < (int) cluster.n_clusters; j++)
        {
            Vector2D center{};
            elements = 0.0;

            for(int i{0}; i < (int) cluster.ranges.size(); i++)
            {
                if(cluster.ranges.at(i).cluster == j)
                {
                    center.x += cluster.ranges.at(i).range * cos(cluster.ranges.at(i).angle);
                    center.y += cluster.ranges.at(i).range * sin(cluster.ranges.at(i).angle);
                    elements +=1.0;
                }
            }
            if (elements > 3.0)
            {    
                center.x /= elements;
                center.y /= elements; 

            }
            else{
                center.x = 0.0;
                center.y = 0.0;
            }
            centroids.push_back(center);
        }

        return centroids;
    }




    ClustersCentroids shift_points(Clusters cluster, std::vector<Vector2D> centroids)
    {
        std::vector<std::vector<arma::vec>> cluster_pts(cluster.n_clusters);
        ClustersCentroids clusters_dropped{};

        for (int i{0}; i < cluster.n_clusters; i++)
        {
            // 2 vectors (x and y) representing x and y coordinates of points in a cluster
            arma::vec x(360);
            arma::vec y(360);
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
        for(int i{0}; i < (int) cluster_pts.size(); i++)
        {
            cluster_pts.at(i).at(0).resize(point_index.at(i));
            cluster_pts.at(i).at(1).resize(point_index.at(i));
        }

        // drop clusters with < 3 points
        Vector2D centroid_i{};
        int index {0};
        for(int i{0}; i < cluster.n_clusters; i++)
        {
            if(centroids.at(i).x == 0.0 && centroids.at(i).y == 0.0){
                continue;
            }
            else
            {
                centroid_i.x = centroids.at(i).x;
                centroid_i.y = centroids.at(i).y;
                clusters_dropped.centroids.push_back(centroid_i);
                clusters_dropped.points.push_back(cluster_pts.at(i));
                index++;
            }
        }

        return clusters_dropped;
    }




    std::vector<Circle> circle_detection(ClustersCentroids clusters)
    {
        int eig_index{-1};
        std::vector<Circle> circles{};

        for(int i{0}; i < (int) clusters.points.size(); i++)
        {
            arma::vec x = clusters.points.at(i).at(0);
            arma::vec y = clusters.points.at(i).at(1);
            arma::vec z = x%x + y%y;
            double z_bar = mean(z);

            arma::mat bigZ(z.n_elem, 4);
            for (int j{0}; j < (int) z.n_elem; j++)
            {
                bigZ(j,0) = z(j);
                bigZ(j,1) = x(j);
                bigZ(j,2) = y(j);
                bigZ(j,3) = 1.0;
            }
            
            arma::mat Hinv = {{0.0, 0.0, 0.0, 0.5},
                              {0.0, 1.0, 0.0, 0.0},
                              {0.0, 0.0, 1.0, 0.0},
                              {0.5, 0.0, 0.0, -2.0*z_bar}};

            arma::mat U, V, A, Y, Sigma, Q, eigvec;
            arma::vec s, A_star, eigval;

            svd(U, s, V, bigZ);
            
            if(s(3) < 1e-12)
            {
                A = V.col(3);
            }
            else if(s(3) > 1e-12)
            {
                Sigma = {{s(0), 0.0, 0.0, 0.0},
                         {0.0, s(1), 0.0, 0.0},
                         {0.0, 0.0, s(2), 0.0},
                         {0.0, 0.0, 0.0, s(3)}};

                Y = V*Sigma*V.t();
                Q = Y*Hinv*Y;
                arma::eig_sym(eigval, eigvec, Q);
                // int eig_index{0};
                for(int i{0}; i < (int) eigval.n_elem; i++)
                {
                    if(eigval(i) > 0.0)
                    {
                        if(eig_index == -1)
                        {
                            eig_index = i;
                        }
                        else
                        {
                            if(eigval(i) < eigval(eig_index))
                            {
                                eig_index = i;
                            }
                        }
                    }
                }
                A_star = eigvec.col(eig_index);
                A = arma::solve(Y, A_star);
            }

            Circle c{};
            c.a = -A(1)/(2*A(0)) + clusters.centroids.at(i).x;
            c.b = -A(2)/(2*A(0)) + clusters.centroids.at(i).y;
            c.R = sqrt((A(1)*A(1) + A(2)*A(2) - 4*A(0)*A(3))/(4*A(0)*A(0)));

            circles.push_back(c);

        }
        
        return circles;
    }


    std::vector<bool> classification(std::vector<Circle> detected_circles)
    {
       
        std::vector<bool> is_circle{};

        for (int i{0}; i < (int) detected_circles.size(); i++)
        {
            if(detected_circles.at(i).R >  .01 && detected_circles.at(i). R < 0.1)
            {
                is_circle.push_back(true);
            }
            else
            {
                is_circle.push_back(false);
            }
        }
        return is_circle;
    }





}