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

This folder holds the documentation pages for the `autonomy-research-testbed` repo.

## `sim/`

This folder holds simulation files.

### `sim/cpp/`

C++ demos are contained here. To add a new demo, place the `.cpp` file in this directory
and add the demo to the `DEMOS` list in
[`CMakeLists.txt`](../../sim/cpp/CMakeLists.txt).

### `sim/python/`

Python demos are contained here.

### `sim/data/`

Data folders for the simulation are put here.

The [`chrono`](./dockerfiles.md#dockersnippetschronodockerfile) service contains the
Chrono data folder, so there is no need to include that folder again here. Instead,
include demo-specific data files.

Ensure, when writing demos, that you set the Chrono data directories correctly.
```python
# demo.py
chrono.SetChronoDataPath("/opt/chrono/share/chrono/data/")
```
```cpp
// demo.cpp
SetChronoDataPath("/opt/chrono/share/chrono/data/");
```

And then to access data files in `sim/data/`, you just pass the string directly. It will
probably be relative to the `sim/python` or `sim/cpp` folders, respectively.
```python
# demo.py
path_to_data_file = "../data/data_file.txt"
```
```cpp
// demo.cpp
path_to_data_file = "../data/data_file.txt";
```

## `workspace/`

See [this page](./ros_workspace.md) for more information.

## `.pre-commit-config.yaml`

[pre-commit](https://pre-commit.com) is a tool that works along with git to run
specific commands just prior to committing. We basically use it as a glorified code
formatter. On each commit, `pre-commit` should be run such that the commands defined
in the `.pre-commit-config.yaml` file are run.

In addition to on commits, `pre-commit` is required to be run in order for PRs to be
merged. This ensures all code in the main branch is formatted.

To automatically run `pre-commit` on _every_ commit, run the following:
```bash
pre-commit install
```

Please see [the official documentation](https://pre-commit.com) for more detailed
information.

## `atk.yml`

This is the `atk` configuration file. See [the ART/ATK documentation](./atk.md) for
detailed information about how the `atk` file is configured. Additionally, please see
[the official `autonomy-toolkit` documentation](https://projects.sbel.org/autonomy-toolkit) for more details regarding how `atk` works.

## `atk.env`

This file contains environment variables that are evaluated at runtime in the `atk.yml`.
You can think of these values as variables that are substituted into the `atk.yml`
placeholders (like `${VARIABLE_NAME}`). See
[the official docker documentation](https://docs.docker.com/compose/environment-variables/set-environment-variables) for a more detailed explanation.

## `requirements.txt`

This file defines required pip packages needed to interact with this repository.
Currently, `autonomy-toolkit` and `pre-commit` are the only requirements. Additional requirements should be put here.
