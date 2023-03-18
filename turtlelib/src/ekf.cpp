#include "turtlelib/ekf.hpp"
#include <cmath>

namespace turtlelib
{

    EKF::EKF(Config q_0, int num_obst, double process_cov, double r)
    : sigma_est(3+2*num_obst, 3+2*num_obst), n{num_obst}, Q(3,3, arma::fill::eye),
      Q_bar(2*num_obst + 3, 2*num_obst + 3), R(2,2, arma::fill::eye),
      I(3+2*num_obst, 3+2*num_obst, arma::fill::eye), q_prev{q_0}, zeta_est(2*num_obst + 3),
      A_c(2*num_obst + 3, 2*num_obst + 3, arma::fill::eye)
    {
        for(int i{0}; i < n; i++)
        {
            izd.push_back(false);
        }
        

        arma::mat sigma_0m(2*n, 2*n);
        double d = 1e10;
        sigma_0m.diag().fill(d);

        
        sigma_est.submat( 3,3, (3+2*n-1),(3+2*n-1) ) = sigma_0m;

        Q = process_cov * Q;
        Q_bar.submat( 0,0, 2,2 ) = Q;
        R = r*R;

    }


    void EKF::prediction(Config q)
    {   
        zeta_prev = zeta_est;
        sigma_prev = sigma_est;

        zeta_est(0) = normalize_angle(zeta_prev(0) + normalize_angle(normalize_angle(q.theta) - normalize_angle(q_prev.theta)));
        // std::cout<< zeta_est(0)<<std::endl;
        zeta_est(1) = zeta_prev(1) + q.x - q_prev.x;
        zeta_est(2) = zeta_prev(2) + q.y - q_prev.y;

        arma::mat A(3,3);
        A(1,0) = -(q.y - q_prev.y);
        A(2,0) = q.x - q_prev.x;
        A_c.submat(0,0, 2,2) = A;

        q_prev = q;

        sigma_est = A_c*sigma_prev*A_c.t() + Q_bar;
    }


    void EKF::initialization(int index, double dx, double dy)
    {
        int m_index = 3 + 2*index;
        double r = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        double phi = atan2(dy,dx);

        //extended
        zeta_est(m_index) = zeta_est(1) + r*cos(normalize_angle(phi + zeta_est(0)));
        zeta_est(m_index+1) = zeta_est(2) + r*sin(normalize_angle(phi + zeta_est(0)));

    }

    void EKF::correction(int index, double dx, double dy)
    {
        int m_index = 3 + 2*index;
        double r = sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        double phi = atan2(dy,dx);

        arma::vec z_real = {r, phi};

        //theoretical measurement (based on estimate)
        double delta_x = zeta_est(m_index) - zeta_est(1);
        double delta_y = zeta_est(m_index+1) - zeta_est(2);
        double d = delta_x*delta_x + delta_y*delta_y;


        arma::vec z_theor = {sqrt(d), normalize_angle(atan2(delta_y, delta_x) - zeta_est(0))};

        // H matrix
        arma::mat H(2, 3+2*n);

        arma::mat h_1 = {{0.0, -delta_x/sqrt(d), -delta_y/sqrt(d)},
                         {-1.0, delta_y/d, -delta_x/d}};

        arma::mat h_2 = {{delta_x/sqrt(d), delta_y/sqrt(d)},
                         {-delta_y/d, delta_x/d}};

        H.submat(0,0, 1,2) = h_1;

        H.submat(0,m_index, 1,m_index+1) = h_2;

        arma::mat K = sigma_est*H.t()*(H*sigma_est*H.t() + R).i();

        zeta_est = zeta_est + K*(z_real - z_theor);

        sigma_est = (I - K*H)*sigma_est;
        
    }

    arma::vec EKF::mah_distance(Circle lmark, int k)
    {
        // H matrix
        int m_index = 3 + 2*k;
        double r = sqrt(std::pow(lmark.a, 2) + std::pow(lmark.b, 2));
        double phi = atan2(lmark.b,lmark.a);
        double delta_x{0.0};
        double delta_y{0.0};

        arma::vec z_real = {r, phi};

        delta_x = zeta_est(m_index) - zeta_est(1);
        delta_y = zeta_est(m_index+1) - zeta_est(2);
        
        double d = delta_x*delta_x + delta_y*delta_y;

        arma::mat H(2, 3+2*n);

        arma::mat h_1 = {{0.0, -delta_x/sqrt(d), -delta_y/sqrt(d)},
                         {-1.0, delta_y/d, -delta_x/d}};

        arma::mat h_2 = {{delta_x/sqrt(d), delta_y/sqrt(d)},
                         {-delta_y/d, delta_x/d}};

        H.submat(0,0, 1,2) = h_1;

        H.submat(0,m_index, 1,m_index+1) = h_2;

        arma::mat Psi = H * sigma_est * H.t() + R;

        arma::vec z_theor = {sqrt(d), normalize_angle(atan2(delta_y, delta_x))};

        arma::vec d_m = (z_real - z_theor).t() * Psi.i() * (z_real - z_theor);

        return d_m;
    }


}