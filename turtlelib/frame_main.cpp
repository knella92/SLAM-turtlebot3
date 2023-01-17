#include "rigid2d.hpp"


int main()
{
    turtlelib::Transform2D T_ab, T_bc;
    //inputs
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;

    //outputs
    std::cout << "T_{a,b}: " << T_ab << std::endl;
    std::cout << "T_{b,a}: " << T_ab.inv() << std::endl;
    std::cout << "T_{b,c}: " << T_bc << std::endl;
    std::cout << "T_{c,b}: " << T_bc.inv() << std::endl;
    std::cout << "T_{a,c}: " << (T_ab*T_bc) << std::endl;
    std::cout << "T_{c,a}: " << (T_ab*T_bc).inv() << std::endl;
    
    


    return 0;
}