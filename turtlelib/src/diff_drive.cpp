#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <stdexcept>

namespace turtlelib
{


    DiffDrive::DiffDrive(double depth, double radius)
        :Tb1{{0.0, depth}, 0.0},
        Tb2{{0.0, -1*depth}, 0.0},
        D{depth}, r{radius},
        hpi00{(-1.0*radius)/(2.0*depth)}, hpi01{(1.0*radius)/(2.0*depth)}, hpi10{radius/2.0}, hpi11{radius/2.0},
        phi_l{0.0}, phi_r{0.0},
        q{0.0,0.0,0.0}
    {
    }

    DiffDrive::DiffDrive(double depth, double radius, double left_pos, double right_pos)
        :Tb1{{0.0, depth}, 0.0},
        Tb2{{0.0, -1*depth}, 0.0},
        D{depth}, r{radius},
        hpi00{(-1.0*radius)/(2.0*depth)}, hpi01{(1.0*radius)/(2.0*depth)}, hpi10{radius/2.0}, hpi11{radius/2.0},
        phi_l{left_pos}, phi_r{right_pos},
        q{0.0,0.0,0.0}
    {
    }

    Wheel_Vel DiffDrive::inverse_kin(Twist2D & Vb) const{
        if (!almost_equal(Vb.v.y,0)){
            throw std::logic_error("y component of body twist must be zero");
        }
        const Twist2D V1 = Tb1.inv()(Vb);
        const Twist2D V2 = Tb2.inv()(Vb);
        const Wheel_Vel phidot{V1.v.x, V2.v.x};
        return phidot;
    }

    Twist2D DiffDrive::forward_kin(double phi_lp, double phi_rp){
        Twist2D Vb{};
        const double u1{phi_lp - phi_l};
        const double u2{phi_rp - phi_r};

        Vb.w = hpi00 * u1 + hpi01 * u2;
        Vb.v.x = hpi10 * u1 + hpi11 * u2;
        Vb.v.y = 0;
        
        Transform2D Tb_bp = integrate_twist(Vb);
        Config dq{Tb_bp.translation().x, Tb_bp.translation().y, Tb_bp.rotation()};
        q.theta += dq.theta;
        q.x += dq.x*cos(q.theta) - dq.y*sin(q.theta);
        q.y += dq.x*sin(q.theta) + dq.y*cos(q.theta);

        return Vb;
    }

}
