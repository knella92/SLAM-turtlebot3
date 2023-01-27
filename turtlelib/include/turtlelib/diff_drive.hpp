#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of Differential Drive robots

#include "rigid2d.hpp"


namespace turtlelib
{

    /// \brief a robot's configuration
    struct Config
    {
        /// \brief the x position of the robot
        double x;

        /// \brief the y position of the robot
        double y;

        /// \brief the orientation of the robot
        double theta;
    };

    /// \brief wheel velocity (m/s)
    struct Wheel_Vel
    {
        /// \brief left wheel velocity (x and y components)
        Vector2D l;

        /// \brief right wheel velocity (x and y components)
        Vector2D r;
    };

    struct Ang_Vel
    {
        /// \brief left wheel rotational velocity
        double phi_ldot;

        /// \brief right wheel rotational velocity
        double phi_rdot;
    }


    /// \brief forward and inverse kinematics function
    class DiffDrive
    {

        private:

            Transform2D Tb1;

            Transform2D Tb2;

        public:

            /// \brief robot's current right wheel position
            double phi_r{};

            /// \brief robot's current left wheel position
            double phi_l{};

            /// \brief robot's current configuration q (x, y, theta)
            Config q{};

            /// \brief initialize transforms
            /// @param depth 
            DiffDrive(double depth);


            Ang_Vel bodytwist_to_wheelmotion(Twist2D & Vb) const;

            /// \brief computes wheel velocities required to make robot move at a given body twist
            /// \param Vb - given body twist
            /// \return - wheel velocities
            Wheel_Vel inverse_kin(Twist2D & Vb) const;

            /// \brief updates robot's configuration given new wheel positions
            /// \param phi_lp - new left wheel position (phi_l')
            /// \param phi_rp - new right wheel position (phi_r')
            /// \return updated configuration
            Config forward_kin(double phi_lp, double phi_rp);


            

    };

}






#endif