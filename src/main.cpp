#include "mujoco.h"
#include <Eigen/Dense>

#include <iostream>

// Globals
static mjModel* m;
static mjData* d;

// Convenience typedefs for interop between mujoco and eigen
typedef Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mjMapMatrix_t;

typedef Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, 1, Eigen::RowMajor>> mjMapVector_t;

void activate_mujoco()
{
    std::string mjkeyPath;
    
    char *home = std::getenv("HOME");
    if (!home) {
        std::cerr << "Error: home directory not found" << std::endl;
        exit(-1);
    }

    mjkeyPath += std::string(home) + "/.mujoco/mjkey.txt";
    mj_activate(mjkeyPath.c_str());

    char error[1000] = "";
    m = mj_loadXML("assets/cassie.xml", NULL, error, 1000);

    if (!m) {
        std::cerr << error << std::endl;
        exit(-1);
    }

    d = mj_makeData(m);
}

int main() 
{
    activate_mujoco();

    double qpos_init[] = { 
         0.0045, 0, 0.4973, 0.9785, -0.0164,  0.0178, -0.2049,
        -1.1997, 0, 1.4267, 0,      -1.5244,  1.5244, -1.5968,
        -0.0045, 0, 0.4973, 0.9786,  0.0038, -0.0152, -0.2051,
        -1.1997, 0, 1.4267, 0,      -1.5244,  1.5244, -1.5968
    };

    mju_copy(&d->qpos[0], qpos_init, 28);

    mj_forward(m, d);

    // Make sure Eigen works
    Eigen::Vector3d v(1,2,3);  // a vector
    Eigen::AngleAxisd rot(0.5*M_PI, Eigen::Vector3d::UnitY()); // rotation of pi/2 radians
    std::cout << "Applying rotation yields:" << std::endl << rot * v << std::endl;
    Eigen::Translation<double,3> tr(Eigen::Vector3d::UnitZ()); // translation in z-direction
    std::cout << "Applying translation yields:" << std::endl << tr * v << std::endl;
    Eigen::Affine3d f = rot * tr; // combine in one transformation
    std::cout << "Applying both yields:" << std::endl <<  f * v << std::endl;
}