cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    copts = ["-Iexternal/mujoco/include"],
    data = ["//assets:assets"],
    linkopts = ["-lGL -lGLEW -lglfw -lpthread"],
    deps = [
        "//src/render:render",
        "@mujoco//:libmujoco", 
        "@mujoco//:mujoco",
        "@eigen",
    ],
)
