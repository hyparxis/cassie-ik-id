cc_library(
    name = "vision_sim",
    srcs = [
        "vision_sim.cpp",
        "vision_sim.hpp"
    ],
    copts = ["-Iexternal/mujoco/include"], # for GLFW
    hdrs = ["vision_sim.hpp"],
    deps = ["@mujoco"], # for GLFW
    visibility = ["//visibility:public"]
)

cc_binary(
    name = "vision_sim_main",
    srcs = ["vision_sim_main.cpp"],
    copts = ["-Iexternal/mujoco/include"],
    data = ["//assets:assets"],
    linkopts = ["-lGL -lpthread"],
    deps = [
        "//src/vision_sim:vision_sim",
        "@mujoco",
        "@eigen"
    ]
)