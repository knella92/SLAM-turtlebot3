#include "rigid2d.hpp"
#include <iostream>


std::ostream & turtlelib::operator<<(std::ostream & os, const Vector2D & v)
{
    os << '[' << v.x << ' ' << v.y << ']' << '\n';
    return os;
}



int main()
{
    const turtlelib::Vector2D v{1,2};
    std::cout << v;
    return 0;
}