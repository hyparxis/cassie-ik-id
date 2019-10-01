cc_library(
    name = "render",
    srcs = [
        "render.cpp",
        "render.hpp"
    ],
    copts = ["-Iexternal/mujoco/include"],
    hdrs = ["render.hpp"],
    deps = ["@mujoco//:mujoco"],
    visibility = ["//visibility:public"],
)