# Launch System

The launch system is used to help spin up all the nodes associated with a given experiement (e.g. simulation, reality). This page describes the file structure and how files are designed.

> [!NOTE]
> The term "orchestrator" is going to be used to describe a launch file that includes other launch files or does house keeping (defines `LaunchConfigurations`, etc.).

## File Structure

The file structure is as follows:

```
autonomy-research-testbed/
├── art_launch/
├── launch_utils/
└── art_<module>_launch/
```

`<module>` here represents the general component that is being launched (e.g. `control`, `perception`, `simulation`, etc.).

Each folder contains a `launch/` folder where all the launch files should be placed.

### File Naming Convention

All launch files end in `.launch.py`. Furthermore, all launch files specific to the a vehicle platform or orchastrator launch files are prefixed with `art_`.

## `art_launch/`

This is where the main launch file is held: [`art.launch.py`](../../workspace/src/common/launch/art_launch/launch/art.launch.py). This file will do a few things.

1. It will first define system wide parameters (e.g. `LaunchConfigurations`, `LaunchDescriptions`, etc.).
2. It will create a [composable node container](https://docs.ros.org/en/galactic/How-To-Guides/Launching-composable-nodes.html).
3. It will include all other orchestration launch files.

## `launch_utils/`

The `launch_utils` folder contains helper functions for creating launch files. These helpers should be used throughout the launch system.

## `art_<module>_launch/`

This folders deal directly with the subfolders defined in the [ROS Workspace](./ros_workspace.md) page. For instance, the [`art_control_launch`](../../workspace/src/common/launch/art_control_launch/) folder contains launch files for the control nodes.

Each folder will have an orchestrator launch file: `art_<module>.launch.py`. This file is responsible for including the other launch files in this folder which are responsible for individual components or nodes.

For instance, the [`art_sensing_launch`](../../workspace/src/common/launch/art_sensing_launch/) folder contains the [`art_sensing.launch.py`](../../workspace/src/common/launch/art_sensing_launch/launch/art_sensing.launch.py) orchestrator launch file. This file includes the [`usb_cam.launch.py`](../../workspace/src/common/launch/art_sensing_launch/launch/usb_cam.launch.py) and the [`xsens.launch.py`](../../workspace/src/common/launch/art_sensing_launch/launch/xsens.launch.py) launch file which is responsible for launching the camera nodes.