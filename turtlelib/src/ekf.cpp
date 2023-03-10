#include "turtlelib/ekf.hpp"
#include <cmath>

namespace turtlelib
{

    EKF::EKF(Config q_0, int num_obst, double process_cov)
    : zeta_est(2*num_obst + 3), sigma_est(3+2*num_obst, 3+2*num_obst), n{num_obst}
     ,Q(3,3, arma::fill::eye), Q_bar(2*num_obst + 3, 2*num_obst + 3)
     ,A_c(2*num_obst + 3, 2*num_obst + 3, arma::fill::eye)

    {
        for(int i{0}; i < n; i++)
        {
            izd.push_back(false);
        }
        
        
        zeta_est(0) = q_0.theta;
        zeta_est(1) = q_0.x;
        zeta_est(2) = q_0.y;

        arma::mat sigma_0m(2*n, 2*n);
        double d = 1e10;
        sigma_0m.diag().fill(d);

        
        sigma_est.submat( 3,3, (3+2*n-1),(3+2*n-1) ) = sigma_0m;

        Q = process_cov * Q;

    }


    void EKF::prediction(Config q)
    {   
        sigma_prev = sigma_est;

        arma::mat A(3,3);
        A(1,0) = -(q.y - zeta_est(2));
        A(2,0) = q.x - zeta_est(1);
        A_c.submat(0,0, 2,2) = A;

        zeta_est(0) = q.theta;
        zeta_est(1) = q.x;
        zeta_est(2) = q.y;

        //constant A because basic kalman filter, plus our situation is like that
        

        Q_bar.submat( 0,0, 2,2 ) = Q;

        sigma_est = A_c*sigma_prev*A_c.t() + Q_bar;
    }


    void EKF::initialization(double index, double dx, double dy)
    {
        int m_index = 3 + 2*index;
        double r = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        double phi = turtlelib::find_angle(dx,dy);

        //extended
        zeta_est(m_index) = zeta_est(1) + r*cos(phi + zeta_est(0));
        zeta_est(m_index+1) = zeta_est(2) + r*sin(phi + zeta_est(0));

        //basic
        // zeta_est(m_index) = zeta_est(1) + x;
        // zeta_est(m_index + 1) = zeta_est(2) + y;
    }

    void EKF::correction(double index, double dx, double dy)
    {
        int m_index = 3 + 2*index;
        double r = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        double phi = turtlelib::find_angle(dx,dy);
        arma::vec z_real = {r, phi};
        // std::cout << z_real(0) << std::endl;
        //extended, one obstacle
        //theoretical measurement (based on estimate)
            //depends on obstacle in question
        arma::vec z_theor = {zeta_est(m_index) - zeta_est(1), zeta_est(m_index+1) - zeta_est(2)};
        std::cout << zeta_est(1) << std::endl;
        // H matrix
        arma::mat H(2,3+2*n);
        H.submat( 0,m_index, 1,m_index+1) = {{1.0, 0.0}, {0.0, 1.0}};

        arma::mat R;
        R.eye(2,2);
        arma::mat K = sigma_est*H.t()*(H*sigma_est*H.t() + R).i();

        zeta_est = zeta_est + K*(z_real - z_theor);
        arma::mat I;
        I.eye(3+2*n, 3+2*n);
        sigma_est = (I - K*H)*sigma_est;
        
    }
}