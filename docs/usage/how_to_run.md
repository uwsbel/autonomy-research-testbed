# How to Run

This page describes how to run the system. Please review all pages in the [design folder](../design/) before continuing. This page assumes you already have read these pages and have knowledge regarding the system design. Furthermore, it is assumed you have reviewed the [`atk` documentation](https://projects.sbel.org/autonomy-toolkit) and are familiar with the `atk` commands.

## Installing the dependencies

There are a few dependencies that need to be installed before running the system. These are listed in the `requirements.txt` file and can be installed with the following command:

```bash
$ pip install -r requirements.txt
```

In addition, you will need to install docker and docker compose. Please refer to the [official documentation](https://www.docker.com/get-started/) for installation details.

## Using the services

This is a quick review of some `atk` concepts and how to do some basic operations.

### Building the services

> [!TIP]
> Explicitly building the services is not actually required. We have pre-built images on [Docker Hub](https://hub.docker.com/repository/docker/uwsbel/art/general) which will be pulled when you use the `up` command.

Let's say you want to build three services: `vnc`, `chrono`, and `dev`. We can build all of these in one go:

```bash
$ atk dev -b -s vnc chrono dev
```

> [!NOTE]
> This may take a long time to complete considering Chrono has to be built with multiple modules enabled.

### Starting the services

To start each service, we can use the `--up` command in `atk`.

```bash
$ atk dev -u -s vnc chrono dev
```

#### Optionals

Note, at this point, we also want to specify any optionals we want to use. Optionals may be changed at attach time (like if it's setting an environment variable), but if you're requesting specific resources (e.g. a nvidia gpu), this _must_ be done at up time.

You can specify optionals through the `--optionals` flag.

```bash
$ atk dev -s <service> <service> --optionals <optional1> <optional2> ...
```

### Attaching to the services

To attach to each service (as in get a shell), we can use the `--attach` command in `atk`.

```bash
$ atk dev -a <service>
```

You can also combine the `-a` flag with the `-u` flag to start and attach to a service in one go:

```bash
$ atk dev -s <service> -ua
```

> [!NOTE]
> The `-a` flag can only takes one service.

## Running the system

This section describes how to actually run some experiements.

### Starting a simulation

To start a simulation, we use the `chrono` service. We'll start it up, attach to it, and run the simulation. We'll need gpus for the demo, so we'll pass that in as an optional.

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

### Building the autonomy stack

We first need to build the autonomy stack before we use it. This can be done using `colcon build`. Furthermore, as noted in the [Workspace Structure doc](./../design/ros_workspace.md#workspacesrccommonmeta), we can use the `--packages-up-to` flag to only build the packages we need.

> [!NOTE]
> The very first time you run `colcon build`, you may need to install the `bluespace_ai_xsens_ros_mti_driver` library. To do that, run the following:
> ```bash
> $ atk dev -ua -s dev --optionals gpus vnc
> WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
> [+] Running 1/1
>  ✔ Container art-dev  Started
> art@art-dev:~/art/workspace$ pushd src/sensing/bluespace_ai_xsens_ros_mti_driver/lib/xspublic && make && popd
> ```

Here, we'll build the packages for the `art_dev_meta` vehicle, but you can replace `art_dev_meta` with any vehicle-specific metapackage.

```bash
$ atk dev -ua -s dev --optionals gpus vnc
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
[+] Running 1/1
 ✔ Container art-dev  Started
art@art-dev:~/art/workspace$ colcon build --symlink-install --packages-up-to art_dev_meta
```

### Run the autonomy stack

To run the autonomy stack, we'll need to spin up and attach to the `dev` container. We'll then launch the system using the `art.launch.py` orchestrator (see the [Launch System doc](../design/launch_system.md) for more details). We'll also pass `use_sim:=True` to the launch command to disable the hardware drivers.

```bash
$ atk dev -ua -s dev --optionals gpus vnc
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
[+] Running 1/1
 ✔ Container art-dev  Started
art@art-dev:~/art/workspace$ ros2 launch art_launch art.launch.py use_sim:=True
```

### Visualizing the output

It's important, when debugging and tracking progress of the simulation, to be able to visualize the output of the simulation.

#### VNC

We've already started up the `vnc` service and specified `vnc` as an optional for the other services, so we can simply connect to the vnc window through the browser to view any GUI's we want.

The `vnc` service will attempt to deploy a vnc server on any port between `8080-8099` on the host. If the port mappings were not a range, `docker compose` would through an error if that port was already in use (like if you had another vnc server running on the host or were using it for another application). You will need to find the port that the `vnc` service is running on. You can do this by running the following command:

```bash
$ atk dev -s vnc -c ps
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
NAME      IMAGE         COMMAND
art-vnc   atk/art:vnc   "sh -c 'set -ex; exec supervisord -c /opt/supervisord.conf'"   vnc       1 min ago   Up 1 min 127.0.0.1:5900->5900/tcp, 127.0.0.1:8085->8080/tcp
```

In this example, you can see the `vnc` service is being mapped from port `8080` in the container to port `8085` on the host (i.e. `HOST:CONTAINER`).

To connect to this, simply navigate to `localhost:8080` in your browser.

> [!NOTE]
> As you can see by the previous command's output, port `5900-5999` may also be mapped. These ports can be used with a vnc client (e.g. [TigerVNC](https://tigervnc.org/)) to connect to the container. This is useful if you want to use a vnc client instead of the browser.

#### X11

Another option for viewing GUI windows is x11. You may need to do some configuring on your system regarding setting up X11 to run propely (like installing [XQuartz](https://www.xquartz.org/) on mac).

You can use x11 by replacing all `vnc` flags in the `--optionals` with `x11` to do this.

> [!NOTE]
> This only works if you're on the same host as which the container is running on. If you're ssh'd into the host, you'll need to use vnc.
