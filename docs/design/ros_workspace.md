# ROS Workspace

This page describes the underlying philosophy of the ROS workspace for the
`autonomy-research-testbed`. For details on how to spin up the ROS nodes, see the
[How to Run](./../usage/how_to_run.md) page.

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

The ROS packages should be organized in a hierarchy that separates the node directories by their overarching purpose. For instance, perception nodes should be placed in the [`perception/`](./../../workspace/src/perception/) subfolder. See [Workspace Structure](#workspace-structure) for a more detailed explanation of all the
subfolders.

Additionally, this principle is meant to describe what goes in a package. Generally
speaking, a package should implement either a single ROS node, a collection of
like-nodes, or define shared utilities/helpers that are used by other packages. For
instance, the [`launch_utils`](./../../workspace/src/common/launch/launch_utils/) package
does not have a node, but implements utilities used by other launch files.

### Principle 2: Metapackages and launch files organize vehicle spin up/tear down

It is certainly possible that there exists multiple ART vehicles each with a different
setup (i.e. different sensors, computational hardware, etc.). Therefore, this principle
helps to define which nodes are created and/or built as it depends on the specific
vehicle platform in use.

First, [metapackages](https://wiki.ros.org/Metapackages) are a new-ish ROS construct which helps define the build dependencies for a specific package. Essentially, a metapackage has no nodes or code. It is an empty package except for a `package.xml` and `CMakeLists.txt` file which define build dependencies. These build dependencies can then be used to directly build nodes/packages for a specific vehicle platform by only using `colcon build` to build that package.

For instance, if a certain vehicle requires packages named `camera_driver`, `lidar_driver`, `perception`, `control`, and `actuation`, you can specify all these packages as `<exec_depend>` in the metapackage. When `colcon build --packages-up-to <metapackage>` is run, the `<exec_depend>` packages are automatically built.

> [!NOTE]
> See the [Metapackages](#metapackages) section for more information.

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

Vehicle platform metapackages are placed here. They should list all packages that should be built when using that specific vehicle platform in the `package.xml` file as an `<exec_depend>`. See []

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

## Metapackages

As mentioned earlier in [Principle 2](#principle-2-metapackages-and-launch-files-organize-vehicle-spin-uptear-down), metapackages are a powerful tool both at buildtime and at runtime. This section outlines how you should use metapackages, as well as how to leverage `<exec_depend>` in other packages.

### Metapackage Structure

A metapackage is simply a package that has no code, but defines build dependencies. Therefore, the structure of a metapackage is very simple.

```
metapackage/
├── CMakeLists.txt
└── package.xml
```

The CMakeLists.txt file is very simple. It should only contain the following:

```cmake
cmake_minimum_required(VERSION 3.5)
project(metapackage)

find_package(ament_cmake REQUIRED)
ament_package()
```

The magic happens in the `package.xml` file, where we'll define the build dependencies. For instance, if we want to build the `camera_driver`, `lidar_driver`, `perception`, `control`, and `actuation` packages, we would define the following:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format2.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>metapackage</name>
  <version>1.0.0</version>
  <description>A package to aggregate all packages for the vehicle.</description>
  <maintainer email="todo@todo.todo">TODO</maintainer>
  <license>TODO: License declaration</license>

  <exec_depend>camera_driver</exec_depend>
  <exec_depend>lidar_driver</exec_depend>
  <exec_depend>perception</exec_depend>
  <exec_depend>control</exec_depend>
  <exec_depend>actuation</exec_depend>
</package>
```

### Using Metapackages

Given the example `CMakeLists.txt` and `package.xml` files above, we can build the metapackage with `colcon build --packages-up-to metapackage`. This will build all the packages listed in the `<exec_depend>` tags.

Furthermore, we can use the metapackage to run `rosdep` on all the packages listed in the `<exec_depend>` tags. For instance, if we want to install all the dependencies for the packages listed in the metapackage, we can run `rosdep install --from-paths src --ignore-src -y"`. However, this may cause issues if we have other packages in the workspace that we don't want to install dependencies for. Therefore, we want to extract the `--packages-up-to` info from `colcon` and only call `rosdep` on those paths. That can be done with the following command:

```bash
$ paths=$(colcon list --packages-up-to art_dev_meta | awk '{print $2}' | tr '\n' ' ') && \
    rosdep install --from-paths $paths --ignore-src -y
```

### Finding `<exec_depend>` Packages

The `<exec_depend>` tag is used by both `rosdep` and `colcon` to determine which packages to install/build. Therefore, it's important to know how to find the packages that should be listed in the `<exec_depend>` tag.

For local packages (i.e. source packages with code being held locally), it's simply whatever the name of that package is (e.g. `camera_driver`, `control`, etc.). If it's an `apt` or `pip` package, you can use the following commands.

> [!NOTE]
> The first time you run `rosdep`, you may need to run `rosdep update`.

```bash
$ rosdep db --filter-for-installers "apt pip" | grep <package_name>
```

The output will appear like this:

```
<key> -> <value>
```

You should put `<key>` in the `<exec_depend>` tag.
