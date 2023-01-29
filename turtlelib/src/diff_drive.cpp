#include "turtlelib/diff_drive.hpp"


turtlelib::DiffDrive::DiffDrive(double depth)
    :Tb1{{0.0, depth}, 0.0},
     Tb2{{0.0, -1*depth}, 0.0}
{
}

turtlelib::Wheel_Vel turtlelib::DiffDrive::inverse_kin(Twist2D & Vb){

    turtlelib::Twist2D V1 = Tb1.inv()(Vb);
    turtlelib::Twist2D V2 = Tb2.inv()(Vb);
    turtlelib::Wheel_Vel v{V1[1], V2[1]};
    return phidot;

}

turtlelib::Config turtlelib::DiffDrive::forward_kin(double phi_lp, double phi_rp){

}


