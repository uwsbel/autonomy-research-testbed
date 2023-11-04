# `atk.yml`

This file describes the `atk.yml` configuration file specific to this repository. For
a more general overview of `autonomy-toolkit` and it's configuration parameters, please
refer to the [official documentation](https://projects.sbel.org/autonomy-toolkit).

`autonomy-toolkit` is simply a wrapper of `docker compose`. As such, the `atk.yml`
is fully compatible with `docker compose`. The main feature of `autonomy-toolkit`
are [Optionals](#optionals).

> [!NOTE]
> For information on how to actually run `atk` for this repo, refer to the
> [How to Run](./how_to_run.md) page.

## Services

For the `autonomy-research-testbed` repo specifically, there are three main service
types: `dev`/`<vehicle>`, `chrono` and `vnc`.

### `dev`/`<vehicle>`

The `dev` and `<vehicle>` services help spin up images/containers that correspond with
development of the autonomy stack. `dev` should be used on non-vehicle platforms (i.e. lab workstations) for common development work. The `<vehicle>` service (where `<vehicle>` corresponds to an actual vehicle, such as `art-1`) is nearly identical to `dev` with vehicle-specific config (such as device exposure, etc.).

### `chrono`

The `chrono` service spins up a container that contains Chrono and is used to run the
simulation. The `chrono` service should really only ever be run on a powerful workstation and not on the vehicle computer. The autonomy stack then can communicate with the simulator using [Networks](#networks) (if on the same host) or over WiFi/Cellular/LAN.

### `vnc`

The `vnc` service spins up a container that allows visualizing GUI windows in a browser
while running commands in a container. It builds on top of NoVNC. Please see
[How to Run](./how_to_run.md#visualizing-the-output) for a detailed usage explanation.

## Optionals

In addition to services, the `atk.yml` defines a few optional configurations. Optionals are useful configurations that are optionally included in the `docker compose` configuration file at runtime.

An example use case is the following. If someone is developing on a Mac (which doesn't have a NVIDIA gpu), attaching a gpu to the container will throw an error considering one doesn't exist. Optionals provide a helpful mechanism to only apply certain configurations when they are desired/supported.

See [How to Run](./how_to_run.md#optionals) for a detailed usage explanation.

## Networks

Another useful tool in `docker compose` are networks. Networks allow containers running on the same host to communicate with one another in a virtualized way (i.e. without communicating explicitly with the host). This means, if there are two containers running on the same host (e.g. `dev` and `chrono`), they can communicate with each other without needing to do any special networking. By default, all containers spawned in this repository are put on the same network.

> [!NOTE]
> The `vnc` service requires a all services to be on the same network to work. For instance, for `dev` to display a window in the `vnc` browser, the environment variable `DISPLAY` should be set to `vnc:0.0` and the `vnc` service should be spun up on the host. Using the default network, the windows will be displayed automatically.
