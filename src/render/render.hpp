#include "mujoco.h"
#include <mutex>

void render(const mjModel* model, mjData* data, std::recursive_mutex& mtx);

