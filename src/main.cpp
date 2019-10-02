#include "render/render.hpp"

#include "mujoco.h"
#include <Eigen/Dense>

#include <iostream>
#include <thread>
#include <chrono>
#include <utility>
#include <set>

// Globals
static mjModel* m;
static mjData* d;

// Convenience typedefs for interop between mujoco and eigen
typedef Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> mjMapMatrix_t;

typedef Eigen::Map<
    Eigen::Matrix<double, Eigen::Dynamic, 1>> mjMapVector_t;


// TODO: better name
// Avoids contention with code that edits/reads global mjData in seperate threads
// (e.g. the rendering thread), provided they share the same mutex
#define THREADSAFE(mtx, mj_func) \
do {                             \
    mtx.lock();                  \
    mj_func;                     \
    mtx.unlock();                \
} while(0)


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

// Splits dof indices into actuated and unactuated dofs
std::pair<std::set<int>, std::set<int>> split_idcs(const mjModel *m)
{ 
    std::set<int> actuated_idcs, unactuated_idcs;

    for(int dof = 0; dof < m->nv; dof++){
        int jnt = m->dof_jntid[dof];
        auto name = mj_id2name(m, mjOBJ_JOINT, jnt);

        // TODO?: Assumes actuator names correspond to joint names
        // Could use trnid instead

        // Joint has an actuator
        if (name != nullptr && mj_name2id(m, mjOBJ_ACTUATOR, name) != -1) 
            actuated_idcs.insert(dof);
        else
            unactuated_idcs.insert(dof);
    }

    return std::make_pair(actuated_idcs, unactuated_idcs);
}


void project_force(const mjModel *m, mjData* d, std::mutex& mtx)
{
    // Constraint Jacobian (assumes dense)
    mjMapMatrix_t G(d->efc_J, m->nv, m->neq*3);

}


void cassie_inverse_dynamics(const mjModel *m, mjData* d, std::mutex& mtx)
{
    mjMapVector_t q(d->qpos, m->nq);
    mjMapVector_t v(d->qvel, m->nv);
    mjMapVector_t v_dot(d->qacc, m->nv);

    mjMapVector_t tau(d->qfrc_inverse, m->nv);

    // No desired acceleration
    v_dot.setZero();

    THREADSAFE(mtx, mj_inverse(m, d));

    mjMapVector_t tau_applied(d->qfrc_applied, m->nv);

    tau_applied = tau;
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

    std::set<int> aidx, uidx;
    std::tie(aidx, uidx) = split_idcs(m);

    std::mutex mtx;
    std::thread render_thread(render, m, d, std::ref(mtx));

    // Give some time for the rendering thread to initialize
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // TODO: use mjmodel timestep here
    typedef std::chrono::duration<int, std::ratio<1, 2000>> framerate;

    // This will slowly diverge from realtime, since sleep_until 
    // is inexact, but that's fine for our purposes
    while (true) {
        auto start_time = std::chrono::steady_clock::now();
        auto end_time = start_time + framerate(1);

        cassie_inverse_dynamics(m, d, mtx);

        THREADSAFE(mtx, mj_step(m, d));

        std::this_thread::sleep_until(end_time);
    }

    render_thread.join();

    mj_deleteData(d); 
    mj_deleteModel(m);
}