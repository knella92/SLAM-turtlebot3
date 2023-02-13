#include "turtlelib/diff_drive.hpp"
#include <cmath>
#include <stdexcept>

namespace turtlelib
{


    DiffDrive::DiffDrive(double track_width, double radius)
     :  D{track_width/2.0}, r{radius},
        Tb1{{0.0,D}, 0.0},
        Tb2{{0.0, -D}, 0.0},
        hpi00{(radius)/(-2.0*D)}, hpi01{(radius)/(2.0*D)}, hpi10{radius/2.0}, hpi11{radius/2.0},
        phi_l{0.0}, phi_r{0.0},
        q{0.0,0.0,0.0}
    {
    }

    DiffDrive::DiffDrive(double track_width, double radius, Config cfg)
     :  D{track_width/2.0}, r{radius},
        Tb1{{0.0,D}, 0.0},
        Tb2{{0.0, -D}, 0.0},
        hpi00{(radius)/(-2.0*D)}, hpi01{(radius)/(2.0*D)}, hpi10{radius/2.0}, hpi11{radius/2.0},
        phi_l{0.0}, phi_r{0.0},
        q{cfg.x,cfg.y,cfg.theta}
    {
    }

    Wheel_Vel DiffDrive::inverse_kin(Twist2D & Vb) const{
        if (!almost_equal(Vb.v.y,0)){
            throw std::logic_error("y component of body twist must be zero");
        }

        Wheel_Vel phidot{};
        phidot.l = (-D*Vb.w + Vb.v.x)/r;
        phidot.r = (D*Vb.w + Vb.v.x)/r;
        return phidot;
    }

    Twist2D DiffDrive::forward_kin(double dphi_l, double dphi_r){
        Twist2D Vb{};

        Vb.w = hpi00*dphi_l + hpi01*dphi_r;
        Vb.v.x = hpi10*dphi_l + hpi11*dphi_r;
        Vb.v.y = 0.0;
        
        Transform2D Tb_bp = integrate_twist(Vb);
        Config dq{Tb_bp.translation().x, Tb_bp.translation().y, Tb_bp.rotation()};
        q.theta += dq.theta;
        q.x += dq.x*cos(q.theta) - dq.y*sin(q.theta);
        q.y += dq.x*sin(q.theta) + dq.y*cos(q.theta);

        return Vb;
    }

    void DiffDrive::update_wheel_pose(double dphi_l, double dphi_r)
    {
        phi_l += dphi_l;
        phi_r += dphi_r;
    }

}
