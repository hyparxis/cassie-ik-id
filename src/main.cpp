#include "render/render.hpp"

#include "mujoco.h"
#include <Eigen/Dense>

#include <iostream>
#include <thread>

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


// Avoids contention with code that edits global mjData in seperate threads
void mj_forward_threadsafe(const mjModel* m, mjData* d, std::mutex& mtx)
{
    mtx.lock();
    mj_forward(m, d);
    mtx.unlock();
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

    mju_copy(&d->qpos[7], qpos_init, 28);
    mj_forward(m, d);

    std::mutex mtx;
    std::thread render_thread(render, m, d, std::ref(mtx));

    std::this_thread::sleep_for(std::chrono::seconds(5));

    d->qpos[0] = 5; // Not truly threadsafe but we'll let it slide
    mj_forward_threadsafe(m, d, mtx);

    render_thread.join();

    mj_deleteData(d); 
    mj_deleteModel(m);
}