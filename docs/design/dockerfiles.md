# Dockerfiles

The `docker/` folder holds the dockerfiles and data files associated with docker
image/container creation. Background regarding docker, images/containers, and
dockerfiles is outside the scope of this document. For more information, please
refer to the [official documentation](https://docs.docker.com).

This folder is structured as follows:

```
docker/
├── data/
├── common/
│   ├── base.dockerfile
│   ├── common.dockerfile
│   └── final.dockerfile
├── snippets/
│   ├── agx.dockerfile
│   ├── chrono-build.dockerfile
│   ├── optix.dockerfile
│   ├── ros.dockerfile
│   └── rosdep.dockerfile
├── chrono.dockerfile
├── dev.dockerfile
└── vnc.dockerfile
```

> [!NOTE]
> This repository was built to accommodate [autonomy-toolkit](https://projects.sbel.org/autonomy-toolkit). For more information regarding specific commands, please see [Workflow](./../usage/development_workflow.md)

## `docker/data/`

This folder holds data files that may be used by dockerfile snippets. For example,
the [`docker/snippets/optix.dockerfile`](../../docker/snippets/optix.dockerfile) requires the OptiX build script; this file should go here.

## `docker/common/`

This subfolder of `docker/` holds common dockerfile code that is shared across _most_
services. It currently contains three dockerfiles.

### `docker/common/base.dockerfile`

This dockerfile helps initialize the docker system as a whole. It defines global `ARGS`,
such as `USERNAME`, `PROJECT`, etc. Furthermore, it will create a user that has the
desired `uid` and `gid` (can be defined through the `USER_UID` and the `USER_GID`
`ARGS`), and will assign any user groups that the user should be apart of.

**IMAGE_BASE**: Used in conjunction with **IMAGE_TAG**; defines the base image which
the custom docker image will be constructed from. The image is constructed using the
following base image: `${IMAGE_BASE}:${IMAGE_TAG}`. An **IMAGE_BASE** of `ubuntu` and an
**IMAGE_TAG** of `22.04` would then build the image from `ubuntu:22.04`.

**IMAGE_TAG**: Used in conjunction with **IMAGE_TAG**. See above for details. An
**IMAGE_BASE** of `ubuntu` and an **IMAGE_TAG** of `22.04` would then build the image
from `ubuntu:22.04`.

**PROJECT**: The name of the project. Synonymous with `project` in docker.

**USERNAME** _(Default: `${PROJECT}`)_: The username to assign to the new user created
in the image.

**USERHOME** _(Default: `/home/${USERNAME}`)_: The home directory for the new user.

**USERSHELL** _(Default: `bash`)_: The shell to use in the container. Bash is
recommended.

**USERSHELLPATH** _(Default: `/bin/${USERSHELL}`)_: The path to the new user's shell.

**USERSHELLPROFILE** _(Default: `${USERHOME}/.${USERSHELL}rc`): The path to the new
user's shell profile.

**USER_UID** _(Default: 1000)_: The user id (User ID -> UID) that the created user is
assigned. In Linux, this must match the system user with which you launch `atk` from.
If it's not assigned correctly, you will have permission issues when trying to edit
files from the host and/or the container. See the [FAQs](./../misc/faq.md#file-permissions)
for more information.

**USER_GID** _(Default: 1000)_: See **USER_UID** above.

**USER_GROUPS** _(Default: "")_: User groups to add to the new user.

### `docker/common/common.dockerfile`

This dockerfile runs command that we can assume most services want, like package
installation.

**APT_DEPENDENCIES** _(Default: "")_: A space separated list of apt dependencies to
install in the image. Installed with `apt install`.

**PIP_REQUIREMENTS** _(Default: "")_: A space separated list of pip dependencies to
install in the image. Installed with `pip install`.

**USER_SHELL_ADD_ONS** _(Default: "")_: Profile shell addons that are directly echoed
into the user shell profile. For instance,
`USER_SHELL_ADD_ONS: "source /opt/ros/${ROS_DISTRO}/setup.bash"` will run
`echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${USERSHELLPROFILE}`.

### `docker/common/final.dockerfile`

This dockerfile runs commands that are expected to be run after all main installation
snippets are run. It will set the `USER` to our new user, set environment variables, and
set the `CMD` to be `${USERSHELLPATH}`.

## `docker/snippets`

This folder contains dockerfile "snippets", or small scripts that are included in
service dockerfiles to build specific packages, such as Chrono or ROS.

### `docker/snippets/chrono-build.dockerfile`

This file builds Chrono from source. It currently builds a non-configurable list of
chrono modules that is listed below:

- `PyChrono`
- `Chrono::VSG`
- `Chrono::Irrlicht`
- `Chrono::Vehicle`
- `Chrono::Sensor`
- `Chrono::Parsers`
- `Chrono::ROS`

Furthermore, it also builds [`chrono_ros_interfaces`](https://github.com/projectchrono/chrono_ros_interfaces). This is required to build `Chrono::ROS`.

**ROS_DISTRO**: The ROS distro to use.

**ROS_WORKSPACE_DIR** _(Default: `${USERHOME}/ros_workspace`)_. The directory to build
`chrono_ros_interfaces` at. Helpful so that you can add custom messages after building
the image. Ensure you copy the changes to the host before tearing down the container
as this is _not_ a volume.

**CHRONO_ROS_INTERFACES_DIR** _(Default: `${ROS_WORKSPACE_DIR}/src/chrono_ros_interfaces`)_: The folder where the `chrono_ros_interfaces` package is actually cloned.

**CHRONO_BRANCH** _(Default: `main`)_: The Chrono branch to build from.

**CHRONO_REPO** _(Default: `https://github.com/projectchrono/chrono.git`)_: The url of
the Chrono repo to clone and build from.

**CHRONO_DIR** _(Default: `${USERHOME}/chrono`)_: The directory to clone chrono to. The
clone is _not_ deleted to allow people to make changes to the build from within the
container. Ensure you copy the changes to the host before tearing down the container
as this is _not_ a volume.

**CHRONO_INSTALL_DIR** _(Default: `/opt/chrono`)_: The path where Chrono is installed.
The user profile is updated to add the python binary directory to `PYTHONPATH` and
the lib directory is appended to `LD_LIBRARY_PATH`.

**REMOVE_OPTIX** _(Default: `false`)_: Whether to remove the optix library after building chrono. This should be `'true'` if you plan to push the image to some public repository.

### `docker/snippets/ros.dockerfile`

To decrease image size and allow easy customization, ROS is installed separately (as
opposed to the usual method of building _on top_ of an official ROS image). This
snippet will install ROS here.

**ROS_DISTRO**: The ROS distro to use.

**ROS_INSTALL_PREFIX** _(Default: `/opt/ros/${ROS_DISTRO}`)_: The install prefix that ROS is installed to. This should be the folder location of the `setup.bash` file. By default, if installed through `apt` it will be `/opt/ros/${ROS_DISTRO}`. If it's pre-installed for tegra images, it's at `/opt/ros/${ROS_DISTRO}/install`.

### `docker/snippets/optix.dockerfile`

This snippet installs optix in the image. It expects an optix script to be present on the host.

**OPTIX_SCRIPT**: The location _on the host_ that the optix script is located at. This
script can be found on NVIDIA's OptiX downloads page. For more information, see the
[FAQs](./../misc/faq.md#optix-install).

### `docker/snippets/rosdep.dockerfile`

`rosdep` is a useful tool in ROS that parses nested packages, looks inside each
`package.xml` for build dependencies (through `<build_depend>`), and installs the
package through the best means (e.g. `apt`, `pip`, etc.). This file will run `rosdep` on
the ROS workspace located within the `autonomy-research-testbed` repository.

**ROS_DISTRO**: The ROS distro to use.

**ROS_WORKSPACE** _(Default: `./workspace`)_: The directory location _on the host_ of
the ROS workspace to run `rosdep` on.

**ROSDEP_METAPACKAGE** _(Default: `""`)_: If provided, `colcon list --packages-up-to $ROSDEP_METAPACKAGE` will be run to determine the paths to run `rosdep` on (i.e. passed to `--from-paths`). If not provided, `--from-paths src` will be used.

**ROSDEP_OS_NAME** _(Default: `"ubuntu"`)_: This is passed (together with `ROSDEP_OS_VERSION`) to `rosdep` at build time. On systems where it is required to build a ros distribution from source (e.g. Humble on AGX), we need to tell `rosdep` explicitly to use the os version which corresponds to the ROS build. The `--os` is passed as follows: `--os ${ROSDEP_OS_NAME}:${ROSDEP_OS_VERSION}`.

**ROSDEP_OS_VERSION** _(Default: `"jammy"`)_: This is passed (together with `ROSDEP_OS_NAME`) to `rosdep` at build time. On systems where it is required to build a ros distribution from source (e.g. Humble on AGX), we need to tell `rosdep` explicitly to use the os version which corresponds to the ROS build. The `--os` is passed as follows: `--os ${ROSDEP_OS_NAME}:${ROSDEP_OS_VERSION}`.

**ROSDEP_SKIP_KEYS** _(Default: `""`)_: The keys to ignore when attempting to run `rosdep`. Some packages may require being built from source and/or are not available through rosdep, so although we still want to specify them as dependencies in the package, we'll ignore them in the build. This is passed to `rosdep` as follows: `rosdep ... --skip-keys "${ROSDEP_SKIP_KEYS}"`.

**ROS_INSTALL_PREFIX** _(Default: `/opt/ros/${ROS_DISTRO}`)_: The install prefix that ROS is installed to. This should be the folder location of the `setup.bash` file. By default, if installed through `apt` it will be `/opt/ros/${ROS_DISTRO}`. If it's pre-installed for tegra images, it's at `/opt/ros/${ROS_DISTRO}/install`.

## `docker/chrono.dockerfile`

The dockerfile for the `chrono` service. It will do the following:

1. Run `base.dockerfile`
2. Install ROS
3. Install Chrono
4. Run `common.dockerfile`
5. Run `final.dockerfile`

## `docker/dev.dockerfile`

The dockerfile for the `dev` service. It will do the following:

1. Run `base.dockerfile`
2. Install ROS
3. Run `rosdep`
4. Run `common.dockerfile`
5. Run `final.dockerfile`

## `docker/vnc.dockerfile`

The dockerfile for the `vnc` service.

## More Information

Below is some additional information for people interested in the underlying workings of the docker implementation.

### `dockerfile-x`

In order to be more extensible and general purpose, the dockerfiles mentioned below were built around `dockerfile-x`. [`dockerfile-x`](https://github.com/devthefuture-org/dockerfile-x) is a docker plugin that supports importing of other dockerfiles through the `INCLUDE` docker build action. Using `INCLUDE`, we can construct service dockerfiles that mix and match different [snippets](#dockersnippets) that we implement.

### Docker Hub

To expedite start up time, multi-platform builds have been made available on [uwsbel's Docker Hub profile](https://hub.docker.com/repository/docker/uwsbel/art/general). These are automatically built using [Github Actions](https://github.com/features/actions) (see the [workflow file](./../../.github/workflows/build-docker-images.yml)).

Leveraging `docker buildx bake`, images can be built on a specific architecture for other achitectures. What this means is that we can use a basic Github runner to build images that can be used on multiple OS's and/or hardware (e.g. MacOS, arm, x86, etc.). This means that when spinning up the images on our system, building the images locally is no longer required. Furthermore, when an update is pushed to the repository, because Github Actions are automated on pushes to `master`, new images will be built and pushed to Docker Hub; retrieving the new images is as easy as `atk dev -c pull -s <service with new image>`.
