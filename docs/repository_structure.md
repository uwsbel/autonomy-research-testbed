# Repository Structure

This page describes how this repository's directories are organized.

This repo is structured as follows:
```
autonomy-research-testbed/
├── docker/
├── docs/
├── sim/
├── workspace/
├── .pre-commit-config.yaml
├── atk.yml
├── atk.env
└── requirements.txt
```

> [!NOTE]
> Some files are excluded from the list above for brevity.

## `docker/`

See [this page](./dockerfiles.md) for more information.

## `docs/`

This folder holds the documentation for the `autonomy-research-testbed` repo.

## `sim/`

This folder holds simulation files.

### `sim/cpp/`

C++ demos are contained here.

### `sim/python/`

Python demos are contained here.

### `sim/data/`

Data folders for the simulation are put here.

> [!NOTE]
> When building the `chrono` service's image, the Chrono's data folder is both contained
> in the Chrono clone directory and in the shared installed directory (see
> [`dockerfiles`](./dockerfiles.md#dockersnippetschronodockerfile) for more
> information). Therefore, the sim files should set the Chrono data directory to one of
> these folders. Additional data files that should be loaded at runtime should be set
> directly (i.e. don't use the Chrono path utilities).

## `workspace/`

See [this page](./ros_workspace.md) for more information.

## `.pre-commit-config.yaml`

[pre-commit](https://pre-commit.com) is a tool that works along with git to run
specific commands just prior to committing. We basically use it as a glorified code
formatter. On each commit, `pre-commit` should be run such that the commands defined
in the `.pre-commit-config.yaml` file are run.

In addition to on commits, `pre-commit` is required to be run in order for PRs to be
merged. This ensures all code in the main branch is formatted.

Please see [the official documentation](https://pre-commit.com) for more detailed
information.

## `atk.yml`

This is the `atk` configuration file. See [the ART/ATK documentation](./atk.md) for
detailed information about how the `atk` file is configured. Additionally, please see
the official `autonomy-toolkit` documentation for more details regarding how `atk` works.

## `atk.env`

This file contains environment variables that are evaluated at runtime in the `atk.yml`.
The values defined here can be thought of as variables that are substituted into the
defined locations in `atk.yml` (e.g. `${VARIABLE_NAME}`). See
[the official docker documentation](https://docs.docker.com/compose/environment-variables/set-environment-variables) for a more detailed explanation.

## `requirements.txt`

This file defines required pip packages needed to interact with this repository.
Currently, `autonomy-toolkit` is the only requirement. Additional requirements should be
put here.
