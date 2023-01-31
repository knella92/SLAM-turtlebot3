#include "turtlelib/diff_drive.hpp"


turtlelib::DiffDrive::DiffDrive(double depth, double radius)
    :Tb1{{0.0, depth}, 0.0},
     Tb2{{0.0, -1*depth}, 0.0},
     D{depth}, r{radius},
     H_pi{{(-1.0*radius)/(2.0*depth), (1.0*radius)/(2.0*depth)}, {radius/2.0, radius/2.0}},
     phi_l{0.0}, phi_r{0.0},
     q{0.0,0.0,0.0}
{
}

turtlelib::DiffDrive::DiffDrive(double depth, double radius, double left_pos, double right_pos)
    :Tb1{{0.0, depth}, 0.0},
     Tb2{{0.0, -1*depth}, 0.0},
     D{depth}, r{radius},
     H_pi{{(-1.0*radius)/(2.0*depth), radius/(2.0*depth)}, {radius/2.0, radius/2.0}},
     phi_l{left_pos}, phi_r{right_pos},
     q{0.0,0.0,0.0}
{
}

turtlelib::Wheel_Vel turtlelib::DiffDrive::inverse_kin(Twist2D & Vb) const{
    if (!almost_equal(Vb.v.y,0)){
        throw std::logic_error("y component of body twist must be zero");
    }
    const turtlelib::Twist2D V1 = Tb1.inv()(Vb);
    const turtlelib::Twist2D V2 = Tb2.inv()(Vb);
    const turtlelib::Wheel_Vel phidot{V1[1], V2[1]};
    return phidot;
}

void turtlelib::DiffDrive::forward_kin(double phi_lp, double phi_rp){
    turtlelib::Twist2D Vb{};
    const double u1{phi_lp - phi_l};
    const double u2{phi_rp - phi_r};

    Vb.w = H_pi[0][0] * u1 + H_pi[0][1] * u2;
    Vb.v.x = H_pi[1][0] * u1 + H_pi[1][1] * u2;
    Vb.v.y = 0;
    
    turtlelib::Transform2D Twb{{q[0],q[1]},q[2]};
    turtlelib::Transform2D Tb_bp = turtlelib::integrate_twist(Vb);
    Twb*=Tb_bp;
    q.x = Twb.translation().x;
    q.y = Twb.translation().y;
    q.w = Twb.rotation();
}


