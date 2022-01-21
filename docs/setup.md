# Setup

The MiniAV Project has three software elements: the CLI, the Python package, and the ROS 2 control stack. The CLI and Python package are installed in the same way. The ROS 2 stack is distributed and utilized through the development environment which is built on top of Docker.

Setup and installation information is provided in this guide.

## Prerequisites

Before you can install the MiniAV package, you will need to install a few packages. Please see the linked installation instructions before continuing.
- [Docker](https://docker.com): [Installation instructions](https://docs.docker.com/get-docker/)
- [docker-compose](https://docs.docker.com/compose/): [Installation instructions](https://docs.docker.com/compose/install/)

Once the [prerequisites](#prerequisites) have been installed, you may proceed to installing the MiniAV package.

## Python Package 

To install the `miniav` Python package, it is fairly simple. 

### Using `pip`

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

You can test the installation by running the demos in [`demos/`](https://github.com/uwsbel/miniav/tree/master/demos). They demonstrate the CLI and the python API.
