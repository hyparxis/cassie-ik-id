cc_import(
    name = "libmujoco",
    hdrs = ["include/mujoco.h"],
    shared_library = "bin/libmujoco200nogl.so",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "mujoco",
    hdrs = glob(["include/*"]),
    includes = ["."],
    visibility = ["//visibility:public"],
)