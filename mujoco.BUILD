cc_import(
    name = "libmujoco",
    hdrs = ["include/mujoco.h"],
    shared_library = "bin/libmujoco200.so",
    visibility = ["//visibility:public"],
)

cc_import(
    name = "libmujoconogl",
    hdrs = ["include/mujoco.h"],
    shared_library = "bin/libmujoco200nogl.so",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "mujoco",
    hdrs = glob(["include/*"]),
    srcs = ["bin/libglew.so", "bin/libglfw.so.3"],
    includes = ["."],
    visibility = ["//visibility:public"],
)