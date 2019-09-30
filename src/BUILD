cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    copts = ["-Iexternal/mujoco/include"],
    deps = [
        "@mujoco//:libmujoco", 
        "@mujoco//:mujoco",
        "@eigen",
    ]
)
