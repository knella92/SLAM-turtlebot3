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
            if (elements > 3.0)
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




    ClustersCentroids shift_points(Clusters cluster, std::vector<Vector2D> centroids)
    {
        // std::vector<Vector2D> centroids = centroid_finder(cluster);
        std::vector<std::vector<arma::vec>> cluster_pts(cluster.n_clusters);
        ClustersCentroids clusters_dropped{};

        for (int i{0}; i < cluster.n_clusters; i++)
        {
            // 2 vectors (x and y) representing x and y coordinates of points in a cluster
            arma::vec x(300);
            arma::vec y(300);
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

        //resize arma vectors
        for(int i{0}; i < (int) point_index.size(); i++)
        {
            cluster_pts.at(i).at(0).resize(point_index.at(i));
            cluster_pts.at(i).at(1).resize(point_index.at(i));
        }

        // drop clusters with < 2 points
        // std::vector<std::vector<arma::vec>> cluster_pts_dropped;
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
        // comput z_i and mean of z_i
        // std::vector<arma::vec> z_all{};
        int eig_index{-1};
        // std::vector<double> z_bar_all{};
        // std::vector<arma::mat> bigZ_all{};
        // std::vector<arma::mat> M_all{};
        // std::vector<arma::mat> Hinv_all{};
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

            arma::mat M = (1.0/((double) z.n_elem)) * bigZ.t() * bigZ;
            
            arma::mat Hinv = {{0.0, 0.0, 0.0, 0.5},
                              {0.0, 1.0, 0.0, 0.0},
                              {0.0, 0.0, 1.0, 0.0},
                              {0.5, 0.0, 0.0, -2.0*z_bar}};

            arma::mat U, V, A, Y, Sigma, Q, eigvec;
            arma::vec s, A_star, eigval;
            // arma::cx_vec eigval;
            // arma::cx_mat eigvec;

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
            // z_bar_all.push_back(z_bar);
            // z_all.push_back(z);
            // bigZ_all.push_back(bigZ);
            // M_all.push_back(M);
            // Hinv_all.push_back(Hinv);
        }
        
        return circles;
    }


    std::vector<bool> classification(std::vector<Circle> detected_circles)
    {
        // std::vector<std::vector<arma::vec>> shifted_points{};
        std::vector<bool> is_circle{};
        // arma::vec std_dev_vec((int) detected_circles.size());
        // arma::vec means(size(std_dev_vec));
        // for(int i{0}; i < (int) clusters.centroids.size(); i++)
        // {
        //     arma::vec x(size(clusters.points.at(i).at(0)));
        //     arma::vec y(size(x));
        //     for(int j{0}; j < (int) clusters.points.at(i).at(0).n_elem; j++)
        //     {
        //         x(j) = clusters.points.at(i).at(0)(j) + clusters.centroids.at(i).x;
        //         y(j) = clusters.points.at(i).at(1)(j) + clusters.centroids.at(i).y;
        //     }
        //     std::vector<arma::vec> points{};
        //     points.push_back(x);
        //     points.push_back(y);
        //     shifted_points.push_back(points);
        // }

        // for (int i{0}; i < (int) clusters.centroids.size(); i++)
        // {
        //     Vector2D P1{clusters.points.at(i).at(0)(0), clusters.points.at(i).at(1)(0)};
        //     Vector2D P2{*clusters.points.at(i).at(0).end(), *clusters.points.at(i).at(1).end()};
        //     double total{0.0};
        //     double elements{0.0};
        //     arma::vec angles(clusters.points.at(i).at(0).n_elem - 2);

        //     for(int j{1}; j < (int) clusters.points.at(i).at(0).n_elem - 1; j++)
        //     {
        //         Vector2D P{clusters.points.at(i).at(0)(j), clusters.points.at(i).at(1)(j)};
        //         Vector2D vector1{P1.x - P.x, P1.y - P.y};
        //         Vector2D vector2{P2.x - P.x, P2.y - P.y};
        //         double angle = turtlelib::angle(vector1, vector2);
        //         total += angle;
        //         elements += 1.0;
        //         angles(j-1) = angle;
        //     }
        //     const auto mean = total/elements;
        //     means(i) = mean;
        //     double sum_dev{0.0};
        //     for (int j{0}; j < (int) angles.n_elem; j++ )
        //     {
        //         sum_dev += std::pow(angles(j) - mean, 2.0);
        //     }
        //     const auto stddev = sqrt(sum_dev/(elements - 1.0));
        //     std_dev_vec(i) = stddev;
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