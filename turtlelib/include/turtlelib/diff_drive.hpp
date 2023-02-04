#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of Differential Drive robots

#include "turtlelib/rigid2d.hpp"


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

    /// \brief struct containing left and right wheel x rotational velocities
    struct Wheel_Vel
    {
        /// \brief left wheel x rotational velocity
        double l;

        /// \brief right wheel x rotational velocity
        double r;
    };


    /// \brief forward and inverse kinematics function
    class DiffDrive
    {

        private:
            /// \brief transform from body frame to wheel 1 (left)
            Transform2D Tb1;

            /// \brief transform from body frame to wheel 2 (right)
            Transform2D Tb2;

            /// \brief depth of the wheels (distance between wheel frame and body frame in y dimension) 
            double D;

            /// \brief radius of the wheels
            double r;

            /// \brief 2D H pseudo-inverse matrix
            std::vector<std::vector<double>> H_pi;



        public:

            /// \brief robot's current left wheel position
            double phi_l;

            /// \brief robot's current right wheel position
            double phi_r;

            /// \brief robot's current configuration q (x, y, theta)
            Config q{};

            /// \brief initialize transforms, D, H pseudo-inverse matrix, set's current wheel positions to zero
            /// \param depth - depth between center of one wheel and center of chassis
            /// \param radius - radius of wheels
            DiffDrive(double depth, double radius);

            /// \brief initialize transforms, D, H pseudo-inverse matrix, set's current wheel positions to given values
            /// \param depth - depth between center of one wheel and center of chassis
            /// \param radius - radius of wheels
            /// \param left_pos - current left wheel position
            /// \param right_pos - current right wheel position
            DiffDrive(double depth, double radius, double left_pos, double right_pos);

            /// \brief computes wheel velocities required to make robot move at a given body twist
            /// \param Vb - given body twist
            /// \return - wheel velocities
            Wheel_Vel inverse_kin(Twist2D & Vb) const;

            /// \brief updates robot's configuration given new wheel positions
            /// \param phi_lp - new left wheel position (phi_l')
            /// \param phi_rp - new right wheel position (phi_r')
            /// \return claculated body twist
            turtlelib::Twist2D forward_kin(double phi_lp, double phi_rp);


            

    };

}






#endif