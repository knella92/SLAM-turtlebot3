#ifndef EKF_INCLUDE_GUARD_HPP
#define EKF_INCLUDE_GUARD_HPP
/// \file   ekf.hpp
/// \brief SLAM package

#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>


namespace turtlelib
{


    /// \brief extended kalman filter
    class EKF
    {

        private:
            //things that don't have to be accessed in the node itself (variables, helper functions)

            arma::vec zeta_0;


           


            arma::vec zeta_c;


            arma::vec zeta_prev;


            arma::mat sigma_est;


            arma::mat sigma_prev;

            int n;
            
            arma::mat Q;


            arma::mat Q_bar;

            arma::mat R;

            arma::mat I;

            Config q_prev;
            



        public:

            arma::vec zeta_est;

            arma::mat A_c;

            std::vector<bool> izd;

            
            
            //things that need to be accessed outside of the node (functions)

            /// \brief initializes state vector (zeta_0) and system covariance
            /// \param q_0 - initial robot configuration
            /// \param m_0 - vector of obstacle positions
            EKF(Config q_0, int num_obst, double process_cov, double r); // must be implemented in initial_pose service


            void prediction(Config q); // currently only for 1D, basic filter


            void initialization(double index, double dx, double dy);


            void correction(double index, double dx, double dy);

    };












}


#endif