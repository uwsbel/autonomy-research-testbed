# ROS Workspace

This page describes the underlying philosophy of the ROS workspace for the
`autonomy-research-testbed`. For details on how to spin up the ROS nodes, see the
[How to Run](./how-to-run.md) page.

> [!NOTE]
> This page assumes some underlying experience with ROS. Please ask a more experienced
> lab member or refer to the [official documentation](https://docs.ros.org) for more
> information.

## Philosophy

The general philosophy of the ROS workspace structure is inspired from
[Autoware Universe](https://github.com/autowarefoundation/autoware.universe.git).
Basically, the philosophy can be split into three main principles.

### Principle 1: ROS packages are separated by function

This principle serves two purposes: it defines how the package folders are organized and
what should be implemented in a package.

The ROS packages should be organized in a hierarchy that separates the node directories by their overarching purpose. For instance, perception nodes should be placed in the [`perception/`](../workspace/src/perception/) subfolder. See [Workspace Structure](#workspace-structure) for a more detailed explanation of all the
subfolders.

Additionally, this principle is meant to describe what goes in a package. Generally
speaking, a package should implement either a single ROS node, a collection of
like-nodes, or define shared utilities/helpers that are used by other packages. For
instance, the [`launch_utils`](../workspace/src/common/launch/launch_utils/) package
does not have a node, but implements utilities used by other launch files.

### Principle 2: Metapackages and launch files organize vehicle spin up/tear down

It is certainly possible that there exists multiple ART vehicles each with a different
setup (i.e. different sensors, computational hardware, etc.). Therefore, this principle
helps to define which nodes are created and/or built as it depends on the specific
vehicle platform in use.

First, [metapackages](https://wiki.ros.org/Metapackages) are a new-ish ROS construct which helps define the build dependencies for a specific package. Essentially, a metapackage has no nodes or code. It is an empty package except for a `package.xml` and `CMakeLists.txt` file which define build dependencies. These build dependencies can then be used to directly build nodes/packages for a specific vehicle platform by only using `colcon build` to build that package.

For instance, if a certain vehicle requires packages named `camera_driver`, `lidar_driver`, `perception`, `control`, and `actuation`, you can specify all these packages as `<build_depend>` in the metapackage. When `colcon build --packages-select <metapackage>` is run, the `<build_depend>` packages are automatically built.

**TL;DR: Each vehicle platform should have a metapackage that defines it's nodes that are required to be built for it to run successfully.**

In a similar vein, individual vehicle platforms should have a launch file which is the
primary entrypoint for which the vehicle nodes can be launched. This main launch file
should include other launch files which are shared between vehicle platforms, as well
as launch files specific to this platform.

### Principle 3:

## Workspace Structure

This subsection describes how the workspace is structured and what should be placed
in each subfolder.

```
workspace/src/
├── common/
├── control/
├── external/
├── localization/
├── path_planning/
├── perception/
├── sensing/
├── simulation/
└── vehicle/
```

> [!NOTE]
> Only non-obvious folders are described below. For instance, it's fairly clear what
> type of package should be placed in `perception/`. For node specific documentation,
> please refer to the package folder readme.

### `workspace/src/common`

Included in this subfolder is common utilities, interfaces, launch files, and
metapackages.

#### `workspace/src/common/interfaces`

An interface in ROS is schema file that defines either a message (`.msg`), action (`.action`), or service (`.srv`). Custom internal messages should be defined here.

#### `workspace/src/common/launch`

Launch files for spinning up the vehicle platforms should be implemented here. For a more detailed explanation about the launch system, please refer to [the Launch System page](./launch_system.md).

#### `workspace/src/common/meta`

Vehicle platform metapackages are placed here.

### `workspace/src/external`

External packages that are used for debug should be placed here. For instance,
`foxglove` or `rosboard` packages should be placed here. Usually, these are submodules.

### `workspace/src/sensing`

Packages placed here are responsible for interfacing with sensors (i.e. drivers).
These are usually submodules and not written by us.

### `workspace/src/simulation`

These packages are used to interface with a simulation platform.

### `workspace/src/vehicle`

This subfolder is similar to `sensing/`, but interfaces with the vehicle specifically
and these packages may not have sensors. For instance, actuation drivers should be
defined here.
