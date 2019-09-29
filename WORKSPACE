load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

# Mujoco
http_archive(
    name = "mujoco",
    url = "https://www.roboti.us/download/mujoco200_linux.zip",
    build_file = "//:mujoco.BUILD",
    strip_prefix = "mujoco200_linux"
)

# Eigen
http_archive(
    name = "eigen",
    url = "https://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz",
    sha256 = "7e84ef87a07702b54ab3306e77cea474f56a40afa1c0ab245bb11725d006d0da",
    strip_prefix = "eigen-eigen-323c052e1731",
    build_file = "//:eigen.BUILD",
)