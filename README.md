# MiniAV Project

The MiniAV project is meant to be a testbed for automated driving algorithm development and is to be used as a mechanism for validating the simulation platform [Chrono](https://projectchrono.org). The MiniAV platform is under activate development by the [Simulation Based Engineering Laboratory](https://sbel.wisc.edu) at the [University of Wisconsin-Madison](https://wisc.edu). 

## Background

The MiniAV platform provides the following resources for expediting development of autonomous algorithms:
1. A parts list and instructions on how to build the physical platform
2. A high-fidelity simulation environment built using [Chrono](https://projectchrono.org) that accurately models the vehicles dynamics and sensors
3. A development workflow for writing ROS 2 code and premade algorithms to control the vehicle through a closed cone course
4. A database system to expedite data recording and parsing and an accompaning command line interface

## Setup

To setup the miniav cli, it is fairly simple. 

### Prerequisites

Before you can install the MiniAV package, you will need to install a few packages. Please see the linked installation instructions before continuing.
- [Docker](https://docker.com): [Installation instructions](https://docs.docker.com/get-docker/)
- [docker-compose](https://docs.docker.com/compose/): [Installation instructions](https://docs.docker.com/compose/install/)

Once the [prerequisites](#prerequisites) have been installed, you may proceed to installing the MiniAV package.

### Using pip

The MiniAV package is available on [PyPI](https://pypi.org/project/miniav). To install it, run the following command:

```bash
pip install miniav
```

### From Sources

Or, you can install the MiniAV package from sources. To do that, clone the miniav repo locally:

```bash
git clone git@github.com:uwsbel/miniav.git
cd miniav
```

Then, use `setuptools` to install the miniav package:

```bash
python setup.py install
```

_**Note: If you're planning on developing the package, you may wish to install it as symlinks:**_

```bash
python setup.py develop
```

### Test the demos

You should now be all set up!

You can test the installation by running the demos in `demos/`. They demonstrate the CLI and the python API.
