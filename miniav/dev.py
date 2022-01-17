"""
CLI command that handles working with the MiniAV development environment
"""

# Imports from miniav
from miniav.utils.logger import LOGGER

# Docker imports
from python_on_whales import docker, exceptions as docker_exceptions

def _run_env(args):
    """
    Entrypoint for the `dev env` command.

    The `env` command essentially wraps `docker-compose` to automatically build, spin-up, attach, and
    tear down the MiniAV development environment.

    The command is completely redundant; it simply combines all the building, starting, attaching, and destroying
    into a single command. It allows users to quickly start and attach to the MiniAV development environment based
    on the docker-compose file.

    There are four possible options that can be used using the `env` subcommand:
    `build`, `up`, `down`, and `attach`. For example, if you'd like to build the container, you'd run 
    the following command:

    ```bash
    miniav dev env --build
    ```

    If you'd like to build, start the container, then attach to it, run the following command:

    ```bash
    miniav dev env --build --up --attach
    # OR
    miniav dev env -b -u -a
    # OR
    miniav dev env -bua
    ```

    If no arguments are passed, this is equivalent to the following command:

    ```bash
    miniav dev env --up --attach
    ```

    If desired, pass `--down` to stop the container. Further, if the container exists and changes are
    made to the repository, the container will _not_ be built automatically. To do that, add the 
    `--build` argument.
    """
    LOGGER.info("Running 'dev env' entrypoint...")

    # Check docker-compose is installed
    assert docker.compose.is_installed()

    # If no command is passed, start up the container and attach to it
    cmds = [args.build, args.up, args.down, args.attach] 
    if all(not c for c in cmds):
        args.up = True
        args.attach = True

    # Get the config
    try:
        config = docker.compose.config()
    except docker_exceptions.DockerException as e:
        if "no configuration file provided: not found" in str(e):
            LOGGER.fatal("No docker-compose.yml configuration was found. Make sure you are running this command in the MiniAV repository.")
            return
        else:
            raise e

    # Complete the arguments
    if not args.dry_run:
        if args.down:
            LOGGER.info(f"Tearing down...")
            docker.compose.down()
        if args.build:
            LOGGER.info(f"Building...")
            docker.compose.build()
        if args.up:
            LOGGER.info(f"Spinning up...")
            docker.compose.up(detach=True)
        if args.attach:
            LOGGER.info(f"Attaching...")
            try:
                name = "miniav-dev"
                usershell = [e for e in docker.container.inspect(name).config.env if "USERSHELL" in e][0]
                shellcmd = usershell.split("=")[-1]
                shellcmd = [shellcmd, "-c", f"{shellcmd}; echo"]
                print(docker.execute(name, shellcmd, interactive=True, tty=True))
            except docker_exceptions.NoSuchContainer as e:
                LOGGER.fatal(f"The containers have not been started. Please run again with the 'up' command.")

def _init(subparser):
    """Initializer method for the `dev` entrypoint

    This entrypoint provides easy access to the MiniAV development environment. The dev environment
    leverages [Docker](https://docker.com) to allow interoperability across operating systems. `docker-compose`
    is used to build, spin up, attach, and tear down the containers. The `dev` entrypoint will basically wrap
    the `docker-compose` commands to make it easier to customize the workflow to work best for MiniAV.

    The primary container, titled `dev` in the `docker-compose.yml` file in the 
    [`miniav` github](https://github.com/uwsbel/miniav), has [ROS 2](https://docs.ros.org/en/galactic/index.html)
    pre-installed. The software stack for the MiniAV vehicle utilizes ROS 2 and will use
    the same container that is used for development. 

    Additional containers may be provided to allow for GUI windows or run simulations.
    """
    LOGGER.debug("Initializing 'dev' entrypoint...")

    # Add a base 
    dev = subparser
    dev.add_argument("-b", "--build", action="store_true", help="Build the env.", default=False)
    dev.add_argument("-u", "--up", action="store_true", help="Spin up the env.", default=False)
    dev.add_argument("-d", "--down", action="store_true", help="Tear down the env.", default=False)
    dev.add_argument("-a", "--attach", action="store_true", help="Attach to the env.", default=False)
    dev.set_defaults(cmd=_run_env)

    # Create some entrypoints for additinal commands
    subparsers = subparser.add_subparsers(required=False)

    # Subcommand that can build, spin up, attach and tear down the dev environment
    env = subparsers.add_parser("env", description="Command to simplify usage of the docker-based development workflow. Basically wraps docker-compose.")
    env.add_argument("-b", "--build", action="store_true", help="Build the env.", default=False)
    env.add_argument("-u", "--up", action="store_true", help="Spin up the env.", default=False)
    env.add_argument("-d", "--down", action="store_true", help="Tear down the env.", default=False)
    env.add_argument("-a", "--attach", action="store_true", help="Attach to the env.", default=False)
    env.set_defaults(cmd=_run_env)

