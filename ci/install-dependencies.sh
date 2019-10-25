BAZEL_VERSION=1.1.0
yes | sudo apt-get install pkg-config zip g++ zlib1g-dev unzip python3
wget https://github.com/bazelbuild/bazel/releases/download/$BAZEL_VERSION/bazel-$BAZEL_VERSION-installer-linux-x86_64.sh
chmod +x bazel-$BAZEL_VERSION-installer-linux-x86_64.sh
yes | ./bazel-$BAZEL_VERSION-installer-linux-x86_65.sh --user