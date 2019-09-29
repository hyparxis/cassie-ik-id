# IK/ID
This repository will contain code for inverse kinematics and inverse dynamics of a Cassie robot, mainly using MuJoCo and Eigen.

This repo is set-up to be built using Bazel (currently 0.29). To build the code run:

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

In VSCode, add the following config to ```launch.json``` under "configurations":

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
