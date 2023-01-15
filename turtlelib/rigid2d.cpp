#include "rigid2d.hpp"
#include <iostream>
#include <string>

// std::ostream & turtlelib::operator<<(std::ostream & os, const Vector2D & v)
// {
//     os << '[' << v.x << ' ' << v.y << ']' << '\n';
//     return os;
// }

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


int main()
{
    turtlelib::Vector2D k{};
    std::cout<<"Enter 2D vector separated by space\n";
    std::cin>>k;
    std::cout<<k.x<<' '<<k.y<<'\n';

    return 0;
}