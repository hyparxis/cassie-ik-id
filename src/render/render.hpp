#pragma once

#include "mujoco.h"
#include <mutex>

void render(const mjModel* model, mjData* data, std::mutex& mtx);

