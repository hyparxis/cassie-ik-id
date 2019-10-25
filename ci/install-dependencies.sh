BAZEL_VERSION=1.1.0

sudo apt-get update

# OpenGL headers
sudo apt-get install libglu1-mesa-dev freeglut3-dev mesa-common-dev

# Bazel + requirements
sudo apt-get install pkg-config zip g++ zlib1g-dev unzip python3
wget https://github.com/bazelbuild/bazel/releases/download/$BAZEL_VERSION/bazel-$BAZEL_VERSION-installer-linux-x86_64.sh
chmod +x bazel-$BAZEL_VERSION-installer-linux-x86_64.sh
./bazel-$BAZEL_VERSION-installer-linux-x86_64.sh --user