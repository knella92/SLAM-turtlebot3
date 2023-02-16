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

    double find_angle(double dx, double dy)
    {
        double radians{};
        double mag = sqrt(std::pow(dx,2) + std::pow(dy,2));
        if(dy > 0){
            radians = acos(dx/mag);
        }
        else if (dx > 0 && dy < 0){
            radians = asin(dy/mag);
        }
        else if (dx < 0 && dy < 0){
            radians = -acos(dx/mag);
        }
        else if (dx > 0 && almost_equal(dy, 0.0)){
            radians = 0.0;
        }
        else if (almost_equal(dx,0.0) && dy > 0){
            radians = PI/2;
        }
        else if (almost_equal(dy,0.0) && dx < 0){
            radians = PI;
        }
        else if (almost_equal(dx,0.0) && dy < 0){
            radians = 3*PI/2.0;
        }
        return radians;
    }

    Config collision_detection(Config q, double collision_radius, std::vector<double> obstacles_x, std::vector<double> obstacles_y, double obstacles_r)
    {
        double R{collision_radius + obstacles_r};
        double xp{}; double yp{};
        double dx{}; double dy{}; double angle{};
        double dr{};
        for(int i = 0; i < (int) obstacles_x.size(); i++)
        {   
            dx = q.x - obstacles_x.at(i);
            dy = q.y - obstacles_y.at(i);
            dr = sqrt(std::pow(dx,2) + std::pow(dy,2));
            if(dr < R)
            {   
                angle = find_angle(dx, dy);
                xp = obstacles_x.at(i) + R*cos(angle);
                yp = obstacles_y.at(i) + R*sin(angle);
                q.x = xp;
                q.y = yp;
                break;
            }
            else{;}
        }

        return q;
    }

}
