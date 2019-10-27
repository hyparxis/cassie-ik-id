# IK/ID [![Build Status](https://travis-ci.com/p-morais/cassie-ik-id.svg?branch=master)]()


This repository will contain code for inverse kinematics and inverse dynamics of a Cassie robot, mainly using MuJoCo and Eigen.

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

### Debugging
To generate a debug build run ```$ bazel build //src/... compilation_mode=dbg```. For convenience there's also a setting commented out in ```.bazelrc``` that will set this as the default compilation mode.

In VSCode, add the following config to ```launch.json``` under "configurations" (provided you have the official C/C++ extension installed):

```json
{
    "name": "(gdb) Launch Bazel",
    "type": "cppdbg",
    "request": "launch",
    "program": "${workspaceFolder}/bazel-bin/src/main",
    "args": [],
    "stopAtEntry": false,
    "cwd": "${workspaceFolder}",
    "environment": [],
    "externalConsole": false,
    "MIMode": "gdb",
    "setupCommands": [
        {
            "description": "Enable pretty-printing for gdb",
            "text": "-enable-pretty-printing",
            "ignoreFailures": true
        }
    ]
}
```

Then simply select ```(gdb) Launch Bazel``` from the debug drop-down in VSCode.
