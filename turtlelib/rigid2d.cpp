#include "rigid2d.hpp"
#include <iostream>
#include <cmath>

std::ostream & turtlelib::operator<<(std::ostream & os, const Vector2D & v)
{
    os << '[' << v.x << ' ' << v.y << ']' << '\n';
    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, Vector2D & v)
{
    int first{0};
    char x{};
    while (std::cin.peek()!='\n')
    {
        x = std::cin.peek();
        if (x == ' ')
        {
            first = 1;
            std::cin.get(x);
        }
        else if (std::isdigit(x)||x == '-')
        {   
            (first==0)?is>>v.x:is>>v.y;
        }
        else
        {
            std::cin.get(x);
        }
    }
    return is;
}


turtlelib::Transform2D::Transform2D()
    : transf{{1,0,0},{0,1,0},{0,0,1}}
{
}

turtlelib::Transform2D::Transform2D(Vector2D trans)
    : transf{{1.0,0.0,trans.x}
            ,{0.0,1.0,trans.y}
            ,{0.0, 0.0, 1.0}}
{  
}

turtlelib::Transform2D::Transform2D(double radians)
    :  transf{{cos(radians),sin(radians)*-1.0, 0.0}
             ,{sin(radians),cos(radians), 0.0}
             ,{0.0, 0.0, 1.0}}
{
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
    : transf{{cos(radians),sin(radians)*-1.0, trans.x}
            ,{sin(radians),cos(radians), trans.y}
            ,{0.0, 0.0, 1.0}}
{
}

turtlelib::Vector2D turtlelib::Transform2D::operator()(turtlelib::Vector2D v) const
{
    turtlelib::Vector2D newv{};
    newv.x = transf[0][0]*v.x + transf[0][1]*v.y + transf[0][2]*1.0;
    newv.y = transf[1][0]*v.x + transf[1][1]*v.y + transf[1][2]*1.0;
    return newv;
}

turtlelib::Transform2D turtlelib::Transform2D::inv() const
{
    turtlelib::Transform2D invT{};

    invT.transf[0][0] = transf[0][0];
    invT.transf[0][1] = transf[0][1]*-1.0;
    invT.transf[0][2] = -1.0*transf[0][2]*transf[0][0] - transf[1][2]*transf[0][1];

    invT.transf[1][0] = transf[1][0]*-1.0;
    invT.transf[1][1] = transf[1][1];
    invT.transf[1][2] = -1.0*transf[1][2]*transf[0][0] + transf[0][2]*transf[0][1];

    invT.transf[2][0] = 0.0;
    invT.transf[2][1] = 0.0;
    invT.transf[2][2] = 1.0;
    
    return invT;
}

turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const Transform2D & rhs)
{
    this->transf[0][0] = this->transf[0][0]*rhs.transf[0][0] + this->transf[0][1]*rhs.transf[1][0] + this->transf[0][2]*rhs.transf[2][0];
    this->transf[0][1] = this->transf[0][0]*rhs.transf[0][1] + this->transf[0][1]*rhs.transf[1][1] + this->transf[0][2]*rhs.transf[2][1];
    this->transf[0][2] = this->transf[0][0]*rhs.transf[0][2] + this->transf[0][1]*rhs.transf[1][2] + this->transf[0][2]*rhs.transf[2][2];

    this->transf[1][0] = this->transf[1][0]*rhs.transf[0][0] + this->transf[1][1]*rhs.transf[1][0] + this->transf[1][2]*rhs.transf[2][0];
    this->transf[1][1] = this->transf[1][0]*rhs.transf[0][1] + this->transf[1][1]*rhs.transf[1][1] + this->transf[1][2]*rhs.transf[2][1];
    this->transf[1][2] = this->transf[1][0]*rhs.transf[0][2] + this->transf[1][1]*rhs.transf[1][2] + this->transf[1][2]*rhs.transf[2][2];

    this->transf[2][0] = this->transf[2][0]*rhs.transf[0][0] + this->transf[2][1]*rhs.transf[1][0] + this->transf[2][2]*rhs.transf[2][0];
    this->transf[2][1] = this->transf[2][0]*rhs.transf[0][1] + this->transf[2][1]*rhs.transf[1][1] + this->transf[2][2]*rhs.transf[2][1];
    this->transf[2][2] = this->transf[2][0]*rhs.transf[0][2] + this->transf[2][1]*rhs.transf[1][2] + this->transf[2][2]*rhs.transf[2][2];

    return *this;
}

turtlelib::Vector2D turtlelib::Transform2D::translation() const
{
    turtlelib::Vector2D out_v{};
    out_v.x = transf[0][2];
    out_v.y = transf[1][2];
    return out_v;
}

double turtlelib::Transform2D::rotation() const
{
    double out_rad{};
    out_rad = acos(transf[0][0]);
    return out_rad;
}

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Transform2D & tf)
{
    os << '[' << tf.transf[0][0] << ' ' << tf.transf[0][1] << ' ' << tf.transf[0][2] << "]\n"
       << '[' << tf.transf[1][0] << ' ' << tf.transf[1][1] << ' ' << tf.transf[1][2] << "]\n"
       << '[' << tf.transf[2][0] << ' ' << tf.transf[2][1] << ' ' << tf.transf[2][2] << "]\n";
    return os;
}

int main()
{
    turtlelib::Vector2D v{1,2};
    turtlelib::Vector2D trans{1,1};
    double radians = 3.14;
    turtlelib::Transform2D transfor{trans, radians};
    std::cout<<transfor.inv();
    return 0;
}