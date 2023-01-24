#include "turtlelib/rigid2d.hpp"


std::ostream & turtlelib::operator<<(std::ostream & os, const Vector2D & v)
{
    os << '[' << v.x << ' ' << v.y << ']';
    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, Vector2D & v)
{
    int first{0};
    char x{};
    while (is.peek()!='\n')
    {
        x = is.peek();
        if (x == ' ')
        {
            first = 1;
            is.get(x);
        }
        else if (std::isdigit(x)||x == '-')
        {   
            (first==0)?is>>v.x:is>>v.y;
        }
        else
        {
            is.get(x);
        }
    }
    return is;
}


turtlelib::Transform2D::Transform2D() // creates an identity matrix
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
    :  transf{{cos(radians),-sin(radians), 0.0}
             ,{sin(radians),cos(radians), 0.0}
             ,{0.0, 0.0, 1.0}}
{
}

turtlelib::Transform2D::Transform2D(Vector2D trans, double radians)
    : transf{{cos(radians),-sin(radians), trans.x}
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
    invT.transf[0][2] = -1.0*transf[0][2]*transf[0][0] - transf[1][2]*transf[1][0];

    invT.transf[1][0] = -1.0*transf[1][0];
    invT.transf[1][1] = transf[1][1];
    invT.transf[1][2] = -1.0*transf[1][2]*transf[0][0] + transf[0][2]*transf[1][0];

    invT.transf[2][0] = 0.0;
    invT.transf[2][1] = 0.0;
    invT.transf[2][2] = 1.0;
    
    return invT;
}

turtlelib::Transform2D & turtlelib::Transform2D::operator*=(const Transform2D & rhs)
{
    turtlelib::Transform2D tmp{};
    tmp = *this; // saving of lhs to temporary struct

    // matrix multiplication
    tmp.transf[0][0] = transf[0][0]*rhs.transf[0][0] + transf[0][1]*rhs.transf[1][0] + transf[0][2]*rhs.transf[2][0];
    tmp.transf[0][1] = transf[0][0]*rhs.transf[0][1] + transf[0][1]*rhs.transf[1][1] + transf[0][2]*rhs.transf[2][1];
    tmp.transf[0][2] = transf[0][0]*rhs.transf[0][2] + transf[0][1]*rhs.transf[1][2] + transf[0][2]*rhs.transf[2][2];

    tmp.transf[1][0] = transf[1][0]*rhs.transf[0][0] + transf[1][1]*rhs.transf[1][0] + transf[1][2]*rhs.transf[2][0];
    tmp.transf[1][1] = transf[1][0]*rhs.transf[0][1] + transf[1][1]*rhs.transf[1][1] + transf[1][2]*rhs.transf[2][1];
    tmp.transf[1][2] = transf[1][0]*rhs.transf[0][2] + transf[1][1]*rhs.transf[1][2] + transf[1][2]*rhs.transf[2][2];

    tmp.transf[2][0] = 0.0;
    tmp.transf[2][1] = 0.0;
    tmp.transf[2][2] = 1.0;

    *this = tmp;
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
    double radians{};

    if (transf[0][0]<0 && almost_equal(transf[1][0],0)) // if cos(theta) < 0 and sin(theta) is essentially zero, raises possibility for PI or -PI
    {
        double sing{};
        (transf[1][0]<0) ? sing=-1.0 : sing=1.0; // if sin(theta)<0, PI is positive, and vice versa
        radians = PI*sing;
    }
    else
    {
        (transf[1][0] == 0) ? radians=acos(transf[1][1]) : radians=asin(transf[1][0]); // if cos(theta) = 0, use acos(cos(theta)), if not, use asin(sin(theta))
    }
    return radians;
}

turtlelib::Twist2D turtlelib::Transform2D::operator()(turtlelib::Twist2D V) const
{
    turtlelib::Twist2D V_new{};
    double adj[3][3]{{1.0, 0.0, 0.0}
              ,{transf[1][2], transf[0][0], transf[0][1]}
              ,{-transf[0][2], transf[1][0], transf[1][1]}};
    V_new.w = V.w;
    V_new.v.x = adj[1][0]*V.w + adj[1][1]*V.v.x + adj[1][2]*V.v.y;
    V_new.v.y = adj[2][0]*V.w + adj[2][1]*V.v.x + adj[2][2]*V.v.y;

    return V_new;
}

std::ostream & turtlelib::operator<<(std::ostream & os, const turtlelib::Transform2D & tf)
{
    os << "deg: " << turtlelib::rad2deg(tf.rotation()) << " x: " << tf.translation().x << " y: " << tf.translation().y;
    return os;
}

std::istream & turtlelib::operator>>(std::istream & is, turtlelib::Transform2D & tf)
{
    turtlelib::Vector2D vec{};
    double deg{};
    int space_pos{0}; // tells you which white space # peek() is at
    char x{};
    int num{0}; // set to 1 if a number is entered first, which triggers commands accordingly
    while (is.peek()!='\n')
    {
        x = is.peek();
        if (x == ' ')
        {
            space_pos = space_pos + 1;
            is.get(x);
        }
        else if (std::isdigit(x)||x == '-')
        {   
            if(space_pos==0)
            {
                num = 1;
                is >> deg;
                space_pos = 2;
            }
            else if(space_pos==1){is>>deg;}
            else if(space_pos==3)
            {
                is>>vec.x;
                if(num==1){space_pos = 4;}
            }
            else if(space_pos==5){is>>vec.y;}
        }
        else
        {
            is.get(x);
        }
    }
    // Next two lines are to flush out the \n from the istream
    x = is.peek();
    is.get(x);
    double radians = turtlelib::deg2rad(deg);
    turtlelib::Transform2D newtf{vec, radians};
    tf = newtf;
    return is;
}

turtlelib::Transform2D turtlelib::operator*(Transform2D lhs, const Transform2D & rhs)
{
    lhs*=rhs;
    return lhs;
}


turtlelib::Vector2D turtlelib::normalize(Vector2D v)
{
    double mag{};
    mag = sqrt(v.x*v.x + v.y*v.y);
    v.x /= mag;
    v.y /= mag;
    return v;
}


double turtlelib::normalize_angle(double rad)
{
    const double rem = std::remainder(rad,(2*turtlelib::PI));
    double x{};
    if (rem > 0)
    {
        if (rem > 0.5)
        {
            x = rem - 0.5;
            rad = -1*turtlelib::PI + x*2*turtlelib::PI;
        }
        else if (rem < 0.5)
        {
            rad = -1*turtlelib::PI + rem*2*turtlelib::PI;
        }
        else
        {
            rad = turtlelib::PI;
        }
        return rad;
    }
    else if (rem < 0)
    {
        if (rem > -0.5)
        {
            rad = -1*turtlelib::PI + rem*2*turtlelib::PI;
        }
        else if (x < -0.5)
        {
            x = rem + 1;
            rad = x*2*turtlelib::PI;
        }
        else
        {
            rad = turtlelib::PI;
        }
        return rad;
    }
    else
    {
        rad = 0;
        return rad;
    }
}


// Twist2D implementations

turtlelib::Twist2D::Twist2D()
    : w{0}, v{0,0}
{
}

turtlelib::Twist2D::Twist2D(double ang_v, Vector2D xy)
    : w{ang_v}, v{xy}
{
}


std::ostream & turtlelib::operator<<(std::ostream & os, const Twist2D & tw)
{
    os << "[" << tw.w << " " << tw.v.x << " " << tw.v.y << "]";
    return os;
}


std::istream & turtlelib::operator>>(std::istream & is, Twist2D & tw)
{
    turtlelib::Vector2D v{};
    double w;
    int space_pos{0};
    char x{};
    x = is.peek();
    is.get(x);
    while (is.peek()!='\n')
    {
        x = is.peek();
        if (x == ' ')
        {
            space_pos += 1;
            is.get(x);
        }
        else if (std::isdigit(x)||x == '-')
        {   
            if (space_pos == 0)
            {
                is >> w;
            }
            else if (space_pos == 1)
            {
                is >> v.x;
            }
            else if (space_pos == 2)
            {
                is >> v.y;
            }
        }
        else
        {
            is.get(x);
        }
    }
    
    x = is.peek();
    is.get(x);
    turtlelib::Twist2D tmp{w, v};
    tw = tmp;
    return is;
}