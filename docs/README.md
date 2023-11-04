# Autonomy Research Testbed Documentation

These docs are meant to be a _succinct_ reference to commands, packages, and any other
information that may be useful to document as it relates to the
`autonomy-research-testbed` platform.

## Table of Contents

1. Design
    1. [Repository Structure](./design/repository_structure.md)
    2. [`atk.yml`](./design/atk.md)
    3. [Dockerfiles](./design/dockerfiles.md)
    4. [ROS Workspace](./design/ros_workspace.md)
    5. [Launch System](./design/launch_system.md)
2. Usage
    1. [Development Workflow](./usage/development_workflow.md)
    2. [How to Run](./usage/how_to_run.md)
3. [Frequently Asked Questions](./misc/faq.md)

## Quick Start

This section provides the main commands necessary to launch various components. Please ensure you understand the previous topics in the [Table of Contents](#table-of-contents) before continuing.

### Install dependencies

Python dependencies are listed in the `requirements.txt` file and can be installed with the following command:

```bash
pip install -r requirements.txt
```

In addition, you will need to install docker and docker compose. Please refer to the [official documentation](https://www.docker.com/get-started/) for installation details.

### Start up vnc

You'll probably want to visualize GUI windows, so start up vnc first. The first time around, the image will need to be build so this may take a little while.

```bash
$ atk dev -u -s vnc
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
[+] Running 1/1
 ✔ Container art-vnc  Started
```

If you see the following warning, **ignore it**. This simply says that substituting the `$DISPLAY` in the `atk.yml` file fails because `$DISPLAY` is unset which is expected. By passing `vnc` as an optional later, this will override the variable.
```bash
WARN[0000] The "DISPLAY" variable is not set. Defaulting to a blank string.
```

> [!NOTE]
> You can also use x11 if you're _not_ ssh'd to the host. Replace all `vnc` flags in the `--optionals` with `x11` to do this.

### Launch the simulation

The first time you start up the chrono service, it will need to build the image. This may take a while.

```bash
$ atk dev -ua -s chrono --optionals gpus vnc
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
[+] Running 1/1
 ✔ Container art-chrono  Started
art@art-chrono:~/art/sim$ cd python
art@art-chrono:~/art/sim/python$ python3 demo_ART_cone.py --track
Running demo_ART_cone.py...
Loaded JSON: /home/art/art/sim/data/art-1/sensors/camera.json
Loaded JSON: /home/art/art/sim/data/art-1/sensors/accelerometer.json
Loaded JSON: /home/art/art/sim/data/art-1/sensors/gyroscope.json
Loaded JSON: /home/art/art/sim/data/art-1/sensors/magnetometer.json
Loaded JSON: /home/art/art/sim/data/art-1/sensors/gps.json
Shader compile time: 5.04626
Initializing rclcpp.
Initialized ChROSInterface: chrono_ros_node.
```

### Run the autonomy stack

The first time you start up the dev service, it will need to build the image. This may take a while.

```bash
$ atk dev -ua -s dev --optionals gpus vnc
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
[+] Running 1/1
 ✔ Container art-dev  Started
art@art-dev:~/art/workspace$ ros2 launch art_launch art.launch.py use_sim:=True
```
