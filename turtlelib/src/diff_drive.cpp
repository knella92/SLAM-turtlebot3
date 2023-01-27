#include "turtlelib/diff_drive.hpp"


turtlelib::DiffDrive::DiffDrive(double depth)
    :Tb1{{0.0, depth}, 0.0},
     Tb2{{0.0, -1*depth}, 0.0}
{
}


turtlelib::Config turtlelib::DiffDrive::forward_kin(double phi_lp, double phi_rp){

}


turtlelib::Wheel_Vel turtlelib::DiffDrive::inverse_kin(Twist2D & Vb){

    for (int i{1}; i<=2; i++)
    {
        
    }

}