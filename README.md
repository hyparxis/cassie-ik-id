# ik-id
This repository contains code for inverse kinematics and inverse dynamics of a Cassie robot, mainly using MuJoCo and Eigen.

This repo is set-up to be built using Bazel (currently 0.29). To build the code run:

```
$ bazel build //src/...
```

Similarly, to run a demo, run:

```
$ bazel run //src:main
```
