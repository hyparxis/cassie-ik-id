#include <Eigen/Dense>
#include <iostream>

int main() 
{
    Eigen::Vector3d v(1,2,3);  // a vector
    Eigen::AngleAxisd rot(0.5*M_PI, Eigen::Vector3d::UnitY()); // rotation of pi/2 radians
    std::cout << "Applying rotation yields:" << std::endl << rot * v << std::endl;
    Eigen::Translation<double,3> tr(Eigen::Vector3d::UnitZ()); // translation in z-direction
    std::cout << "Applying translation yields:" << std::endl << tr * v << std::endl;
    Eigen::Affine3d f = rot * tr; // combine in one transformation
    std::cout << "Applying both yields:" << std::endl <<  f * v << std::endl;
}