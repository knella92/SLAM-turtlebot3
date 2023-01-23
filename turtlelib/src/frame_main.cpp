#include "turtlelib/rigid2d.hpp"


int main()
{
    turtlelib::Transform2D T_ab, T_bc;
    turtlelib::Vector2D v{};
    turtlelib::Twist2D V_b{};
    
    std::cout << "Enter transform T_{a,b}:" << std::endl;
    std::cin >> T_ab;
    std::cout << "Enter transform T_{b,c}:" << std::endl;
    std::cin >> T_bc;

    std::cout << "T_{a,b}: " << T_ab << std::endl;
    std::cout << "T_{b,a}: " << T_ab.inv() << std::endl;
    std::cout << "T_{b,c}: " << T_bc << std::endl;

    std::cout << "T_{c,b}: " << T_bc.inv() << std::endl;
    std::cout << "T_{a,c}: " << (T_ab*T_bc) << std::endl;
    std::cout << "T_{c,a}: " << (T_ab*T_bc).inv() << std::endl;


    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> v.x >> v.y;
    std::cout << "v_bhat: " << turtlelib::normalize(v)<<std::endl;
    std::cout << "v_a: " << T_ab(v) << std::endl;
    std::cout << "v_b: " << v << std::endl;
    std::cout << "v_c: " << T_bc.inv()(v) << std::endl;

    std::cout << "Enter twist V_b:" << std::endl;
    std::cin >> V_b;
    std::cout << "V_a " << T_ab(V_b) << std::endl;
    std::cout << "V_b " << V_b << std::endl;
    std::cout << "V_c " << T_bc.inv()(V_b) << std::endl;
    


    return 0;
}