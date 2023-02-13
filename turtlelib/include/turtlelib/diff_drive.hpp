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
            /// \brief depth of the wheels (distance between wheel frame and body frame in y dimension) 
            double D;

            /// \brief radius of the wheels
            double r;

            /// \brief transform from body frame to wheel 1 (left)
            Transform2D Tb1;

            /// \brief transform from body frame to wheel 2 (right)
            Transform2D Tb2;

            
 

            /// \brief Moore-Penrose pseudomatrix element [0][0]
            double hpi00;
            
            /// \brief Moore-Penrose pseudomatrix element [0][1]
            double hpi01;

            /// \brief Moore-Penrose pseudomatrix element [1][0]
            double hpi10;

            /// \brief Moore-Penrose pseudomatrix element [1][1]
            double hpi11;



        public:

            /// \brief robot's current left wheel position
            double phi_l;

            /// \brief robot's current right wheel position
            double phi_r;

            /// \brief robot's current configuration q (x, y, theta)
            Config q{};

            /// \brief initializes DiffDrive object
            /// \param track_width - width between wheel frame origins
            /// \param radius - radius of wheels
            DiffDrive(double track_width, double radius);

            /// \brief initialize transforms, D, H pseudo-inverse matrix, sets initial configuration to given parameters
            /// \param track_width - width between wheel frame origins
            /// \param radius - radius of wheels
            /// \param cfg = initial body configuration of robot
            DiffDrive(double track_width, double radius, Config cfg);

            /// \brief computes wheel velocities required to make robot move at a given body twist
            /// \param Vb - given body twist
            /// \return - wheel velocities
            Wheel_Vel inverse_kin(Twist2D & Vb) const;

            /// \brief updates robot's configuration given change in wheel positions
            /// \param dphi_l - change in left wheel position
            /// \param dphi_r - change in right wheel position
            /// \return claculated body twist
            turtlelib::Twist2D forward_kin(double dphi_l, double dphi_r);

            /// \brief updates robot's wheel positions (radians)
            /// \param dphi_l - change in left wheel position
            /// @param dphi_r - change in right wheel position
            void update_wheel_pose(double dphi_l, double dphi_r);

    };

}






#endif