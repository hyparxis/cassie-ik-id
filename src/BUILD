cc_binary(
    name = "main",
    srcs = ["main.cpp"],
    copts = ["-Iexternal/mujoco/include"],
    data = ["//assets:assets"],
    linkopts = ["-lGL -lpthread"],
    deps = [
        "//src/render:render",
        "@mujoco",
        "@eigen",
    ],
)
