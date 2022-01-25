# Using the MiniAV Development Environment

The MiniAV development environment has been created to expedite the process from algorithm implementation to testing in simulation to deploying the control stack on the MiniAV (or any other compatible platform). 

## Prerequisites

- You have cloned the [miniav](https://github.com/uwsbel/miniav) repository
- You have installed Docker ([resource for that](https://docs.docker.com/get-docker/))
- You have installed docker-compose ([resource for that](https://docs.docker.com/compose/install/))
- You have installed `miniav` ([resources for that](https://projects.sbel.org/miniav/setup.html))

## Additional Prerequisites For Linux Users

- You have installed Docker compose v2 ([resource for that](https://docs.docker.com/compose/cli-command/))
- You can run Docker as a non-root user and have activated the Docker daemon ([resource for that](https://docs.docker.com/engine/install/linux-postinstall/))

## Design Considerations

The _most_ important component we considered when creating the development environment was whether the workflow was usable on multiple platforms, i.e. it would work as is on Windows, MacOS, and Linux systems. This was a nonnegotiable because then the MiniAV platform could be developed anywhere and wouldn't require any special hardware or "hacking" to work on a specific system. Further, the development environment and deployment environment (the system that runs on the actual vehicle) must also be the same or similar in design. This means that any customization to the dependency lists or sensor configurations that was made locally would carry over to the actual vehicle.

Another important element we considered was using simulation to test the control stack. [Chrono](https://projectchrono.org) is the simulator of choice and it needed to interface with the development _and_ deployment environment natively. The control stack itself should not be limiting hardware-wise (unless the implemented algorithms require a certain type of CPU, for example), but the Chrono simulations may require specific hardware, such as a NVIDIA GPU. Therefore, the solution must be able to communicate over the network considering everyone may not have access to a NVIDIA GPU on their computer, but a remote server/workstation may.

## Background

To begin, Docker is a tool for virtualizing applications from a main operating system. What this means is that you can run full OS containers within a host OS. The primary purpose behind Docker, and similar tools, is to isolate development environments and to consistently deploy applications across computers. Docker is typically used on servers (think AWS or Azure) to isolate users and to deploy websites and web apps. Docker simply provides the ability to run these isolated containers, it is the users job to create the content that goes inside the containers. For more information on Docker, plesae see their [official website](https://www.docker.com/).

For robotics, containers can be a valuable tool for creating consistent development environments for users with different operating systems or different use cases. For example, a Docker container can be generated that has the entire simulation platform already installed; then, the user can simply run their simulation script in the container without the need to install any dependencies.

To help facilitate complicated scenarios, it is common practice to utilize multiple containers. Think, for instance, with multiple containers, you can have multiple independent systems that can be interchanged easily. Then, each isolated container communicates with the others in some capacity. This is what we will do here, where we have one container for the control stack, another for the simulation, and then other optional containers with other desired features: for example, `vnc` for visualizing gui apps.

To implement the system we've discussed, `docker-compose` is utilized. The `docker-compose.yml` file located at the root of the `miniav` repository is displayed below.

```yaml
version: "3.9"
services:
  dev:
    container_name: 'miniav-dev'
    hostname: 'miniav-dev'
    image: 'sbel/miniav:dev'
    build:
      context: ./
      dockerfile: ./docker/dev/dev.dockerfile
      network: host
      args:
        CONTEXT: docker/dev
        REPONAME: miniav
        ROSDISTRO: galactic
    volumes:
      - .:/root/miniav
    environment:
      - DISPLAY=novnc:0.0
    working_dir: /root/
    tty: true
    networks:
      - miniav
  vnc:
    container_name: 'miniav-vnc'
    hostname: 'miniav-vnc'
    build:
      context: ./docker/vnc/
      dockerfile: ./vnc.dockerfile
      network: host
    environment:
      - RUN_XTERM=no
    ports:
      - "8080:8080"
      - "5900:5900"
    networks:
      - miniav
networks:
  miniav:
```

As it can be seen, there are two `services`: `dev` and `vnc`. `dev` is the ROS 2 development environment we'll use to write the ROS 2 code. `vnc` is the container used to visualize gui apps. Various attributes are included in the `.yml` file, such as build context, ROS version types, and environment variables. As seen in the `volumes` section under the `dev` service, the entire `miniav` repository will be mounted inside the container. A [volume](https://docs.docker.com/storage/volumes/) is simply a folder that is shared between the host OS and the container. This means any and all code additions should be made _only_ inside of this folder; if you edit any files outside of `/root/miniav`, then the changes will not be saved when the container is exited.

## Setup

Beyond installing the packages outlined in [prerequisites](#prerequisites), there is not much setup that is necessary. The `miniav` package provides tools for easily spinning up containers and attaching to the development environment within Docker.

## Usage

To use the development environment, very convenient commands are provided through the `miniav` CLI. The documentation for the `dev` command can be found [here](http://projects.sbel.org/miniav/usage/cli.html#dev).

As described in the documentation, the `dev` command has four arguments: `build`, `up`, `down`, and `attach`. These may sound familiar if you've used `docker-compose` before because the `dev` command essentially wraps `docker-compose`. Everything that the `miniav dev` command does, `docker-compose` can also do; the `miniav dev` cli command is simply made to expedite the process of entering a container and may also provide an easy mechanism to add additional functionality in the future.

```{note}
For any commands mentioned herein, it will be assumed they are run from within the `miniav` repository.
</div></div>
```

### Entering the Development Environment

The first time you attempt to use the MiniAV development environment, the docker container will need to be built (the `miniav` package will do this for you). This may take upwards of 15 minutes, depending on the number of packages your control stack needs to install. After the initial build, you may never need to build the stack again (unless you need additional packages installed). To build the container the first time around, you can run the following command:

```bash
miniav dev
```

This is equivalent to running the following:

```bash
miniav dev --up --attach
```

And since the image has never been built, the `--up` argument will also build it. This is only the case if the image cannot be found, i.e. the first time you run `miniav dev`.

If you make changes to the workspace and need additional packages to be installed into the container, you can run the same command with the build flag:

```bash
miniav dev --build
```

```{note}
This is _not_ equivalient to `miniav dev --up --attach --build`. `--up` and `--attach` are only added if no other arguments are provided
</div></div> 
```

```{warning}
If the container is already running (i.e. `miniav dev --up` has already been called), the new built image will not be loaded automatically. You need to tear down the container by running `miniav dev --down` or `miniav dev -d` and then spin up the container again with `miniav dev --up`
</div></div> 
```

After you run `miniav dev`, you should see a shell prompt like the following:

```bash
$ miniav dev
WARNING  | logger.set_verbosity :: Verbosity has been set to WARNING
miniav-dev:~$
```

### Developing Inside the Container

Once inside the container, if you type `ls`, you should see the `miniav` folder that is a mapped volumes from the host system. As mentioned before, any code you write should be done here or your changes will persist once the container is destroyed.

```{todo}
To write more ...
```

## Support

Contact the [Simulation Based Engineering Laboratory](mailto:negrut@wisc.edu) for any questions or concerns regarding the contents of this repository.

## See Also

Visit our website at [sbel.wisc.edu](https://sbel.wisc.edu)!
