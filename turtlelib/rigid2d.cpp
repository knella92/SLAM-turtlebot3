#include "rigid2d.hpp"
#include <iostream>

int main(){
    bool x = turtlelib::almost_equal(3,3);
    std::cout << (x==true?"true\n":"false\n");
    return 0;
}