# IK/ID [![Build Status](https://travis-ci.com/p-morais/cassie-ik-id.svg?branch=master)](https://travis-ci.com/p-morais/cassie-ik-id) [![Documentation Status](https://readthedocs.org/projects/cassie-ik-id/badge/?version=latest)](https://cassie-ik-id.readthedocs.io/en/latest/?badge=latest)

This repository will contain code for inverse kinematics and inverse dynamics of a Cassie robot, mainly using MuJoCo and Eigen. Additional information about the math behind the code can be found in the [docs](https://cassie-ik-id.readthedocs.io/en/latest/) which contain nicely formatted LaTeX equations.

### Installation 

This repo is set-up to be built using [Bazel](https://docs.bazel.build/versions/master/install.html) (currently 1.1.0). To install the project dependencies (including bazel) run:
```
$ ./scripts/install-dependencies.sh
```
Then to build the project run:

```
$ bazel build //src/...
```

Similarly, to run a demo, run:

```
$ bazel run //src:main
```

Bazel will automatically download Eigen and MuJoCo, but you will still need a MuJoCo license under ```~/.mujoco/mjkey.txt``` to run the code.
