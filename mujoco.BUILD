cc_library(
    name = "mujoco",
    hdrs = glob(["include/*"]),
    srcs = ["bin/libglew.so", "bin/libglfw.so.3", 
            "bin/libmujoco200nogl.so", "bin/libmujoco200.so"],
    includes = ["."],
    visibility = ["//visibility:public"],
)