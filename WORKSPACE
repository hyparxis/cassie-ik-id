load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Mujoco
http_archive(
    name = "mujoco",
    url = "https://www.roboti.us/download/mujoco200_linux.zip",
    strip_prefix = "mujoco200_linux",
    build_file = "//:mujoco.BUILD",
)

# Eigen
http_archive(
    name = "eigen",
    url = "https://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz",
    sha256 = "7e84ef87a07702b54ab3306e77cea474f56a40afa1c0ab245bb11725d006d0da",
    strip_prefix = "eigen-eigen-323c052e1731",
    build_file = "//:eigen.BUILD",
)

# GTest
http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.8.1.tar.gz",
    sha256 = "9bf1fe5182a604b4135edc1a425ae356c9ad15e9b23f9f12a02e80184c3a249c",
    strip_prefix = "googletest-release-1.8.1",
    build_file = "@//:gtest.BUILD",
)

