cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    copts = ["-Iexternal/mujoco/include"],
    data = ["//assets:assets"],
    deps = [
        "@mujoco//:libmujoco", 
        "@mujoco//:mujoco",
        "@eigen",
    ]
)
