#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file   ekf.hpp
/// \brief SLAM package

#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>


namespace turtlelib
{
    /// \brief Circle parameters
    struct Circle
    {
        /// \brief x coordinate of circle
        double a;

        /// \brief y coordinate of circle
        double b;

        /// \brief radius of circle
        double R;
    };

    /// \brief extended kalman filter
    class EKF
    {

        private:
            //things that don't have to be accessed in the node itself (variables, helper functions)

            /// \brief - previous state vector 
            arma::vec zeta_prev;

            /// \brief - current sigma estimate matrix
            arma::mat sigma_est;

            /// \brief - previous sigma estimate matrix
            arma::mat sigma_prev;

            /// \brief - number of obstacles 
            int n;
            
            /// \brief - process covariance
            arma::mat Q;

            /// \brief - process covariance matrix
            arma::mat Q_bar;

            /// \brief - covariance matrix
            arma::mat R;

            /// @brief - Identity matrix
            arma::mat I;

            /// \brief - previous odometry reading 
            Config q_prev;
            

        public:

            /// \brief - current state vector estimate
            arma::vec zeta_est;

            /// \brief - current A matrix 
            arma::mat A_c;

            /// \brief - vector of bool values stating if specific obstacle is initialized
            std::vector<bool> izd;

            
            std::vector<double> obst_radii;


            /// \brief initializes state vector (zeta_0) and system covariance
            /// \param q_0 - initial robot configuration
            /// \param num_obst - number of obstacles to initialize
            /// \param process_cov - Q parameter (process covariance)
            /// \param r - covariance for R matrixx
            EKF(Config q_0, int num_obst, double process_cov, double r); // must be implemented in initial_pose service

            /// \brief - creates zeta_estimate prediction (state vector)
            /// @param q - current odometry reading
            void prediction(Config q); // currently only for 1D, basic filter

            /// \brief - initializes obstacle 
            /// \param index - obstacle id
            /// \param dx - measured distance from robot to obstacle (x)
            /// \param dy - measured distance from robot to obstacle (y)
            void initialization(int index, double dx, double dy);

            /// \brief - corrects the predicted estimate of state vector
            /// \param index - obstacle id
            /// \param dx - measured distance from robot to obstacle (x)
            /// \param dy - measured distance from robot to obstacle (y)
            void correction(int index, double dx, double dy);


            arma::vec mah_distance(Circle lmark, int k);

    };












}


#endif