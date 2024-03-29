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

The primarily dependency for `autonomy-research-testbed` is the `autonomy-toolkit`. See the [official documentation](https://projects.sbel.org/autonomy-toolkit/) for more details. It can be installed with `pip`, so see below.

Python dependencies are listed in the `requirements.txt` file and can be installed with the following command:

```bash
pip install -r requirements.txt
```

In addition, you will need to install docker and docker compose. Please refer to the [official documentation](https://www.docker.com/get-started/) for installation details.

#### Download Optix

To build the chrono image, you'll need to download the OptiX 7.7 build script from NVIDIA's website and place it in [`docker/data`](./../docker/data). You can find the download link [here](https://developer.nvidia.com/designworks/optix/download). See the [FAQs](./misc/faq.md#optix-install) for more details.

#### Download Waypoints Data Files
To run the waypoints-based path following demo, we need to download some waypoints paths as reference trajectories into [`sim/data`](./../sim/data), as well as into the [`workspace/src/path_planning/waypoints_path_planner/waypoints_path_planner`](../workspace/src/path_planning/waypoints_path_planner/waypoints_path_planner/) folder. You can use the following command to download the data files:
```bash
cd <path_to_autonomy_research_testbed_sim_or_path_planner_folder> && gdown --folder 1as3pPYlC0m9LcRJuuVOzuAOYPMlosMfY
```

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

### Build and run the autonomy stack

The first time you start up the dev service, it will need to build the image. This may take a while.

> [!NOTE]
> The very first time you run `colcon build`, you may need to install the `bluespace_ai_xsens_ros_mti_driver` library. To do that, run the following:
> ```bash
> $ atk dev -ua -s dev --optionals gpus vnc
> WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
> [+] Running 1/1
>  ✔ Container art-dev  Started
> art@art-dev:~/art/workspace$ pushd src/sensing/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd
> ```

```bash
$ atk dev -ua -s dev --optionals gpus vnc
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
[+] Running 1/1
 ✔ Container art-dev  Started
art@art-dev:~/art/workspace$ colcon build --symlink-install --packages-up-to art_dev_meta
art@art-dev:~/art/workspace$ ros2 launch art_launch art.launch.py use_sim:=True
```

See the [How to Run](./usage/how_to_run.md) page for more details.
