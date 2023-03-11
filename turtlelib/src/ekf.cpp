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
        double phi = find_angle(dx,dy);

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
        double phi = find_angle(dx,dy); 
        // std::cout << phi << std::endl;
        arma::vec z_real = {r, phi};
        //extended, one obstacle
        //theoretical measurement (based on estimate)
            //depends on obstacle in question
        // arma::vec z_theor = {zeta_est(m_index) - zeta_est(1), zeta_est(m_index+1) - zeta_est(2)};
        double delta_x = zeta_est(m_index) - zeta_est(1);
        double delta_y = zeta_est(m_index+1) - zeta_est(2);
        double d = delta_x*delta_x + delta_y*delta_y;
        double theta_est{};

        if(zeta_est(0) < 0 && zeta_est(0) >= -PI)
        {
            theta_est = zeta_est(0);
        }
        else if(zeta_est(0) < -PI && almost_equal(normalize_angle(zeta_est(0)),PI))
        {
            std::cout << "edge-case" << std::endl;
            theta_est = -PI;
        }
        else{ 
            theta_est = normalize_angle(zeta_est(0));
        
        }
        arma::vec z_theor = {sqrt(d), normalize_angle(find_angle(delta_x, delta_y) - theta_est)};
        // std::cout << zeta_est(1) << std::endl;
        // H matrix
        arma::mat H(2, 3+2*n);

        arma::mat h_1 = {{0.0, -delta_x/sqrt(d), -delta_y/sqrt(d)},
                         {-1.0, delta_y/d, -delta_x/d}};

        arma::mat h_2 = {{delta_x/sqrt(d), delta_y/sqrt(d)},
                         {-delta_y/d, delta_x/d}};

        H.submat(0,0, 1,2) = h_1;

        if(index == 0)
        {
            // std::cout << arma::size(h_2) << std::endl;
            H.submat(0,3, 1,4) = h_2;
        }
        else
        {
            H.submat(0,3+2*index,   1,(3+2*index+1)) = h_2;
        }

        arma::mat R;
        R.eye(2,2);
        arma::mat K = sigma_est*H.t()*(H*sigma_est*H.t() + R).i();

        zeta_est = zeta_est + K*(z_real - z_theor);
        arma::mat I;
        I.eye(3+2*n, 3+2*n);
        sigma_est = (I - K*H)*sigma_est;
        
    }
}