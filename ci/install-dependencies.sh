BAZEL_VERSION=1.1.0

sudp apt-update
apt-get install libmesa-dev
sudo apt-get install pkg-config zip g++ zlib1g-dev unzip python3
wget https://github.com/bazelbuild/bazel/releases/download/$BAZEL_VERSION/bazel-$BAZEL_VERSION-installer-linux-x86_64.sh
chmod +x bazel-$BAZEL_VERSION-installer-linux-x86_64.sh
./bazel-$BAZEL_VERSION-installer-linux-x86_64.sh --user