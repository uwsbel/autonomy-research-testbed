# Setup

The MiniAV Project has three software elements: the CLI, the Python package, and the ROS 2 control stack. The CLI and Python package are installed in the same way. The ROS 2 stack is distributed and utilized through the development environment which is built on top of Docker.

Setup and installation information is provided in this guide.

## Prerequisites

Before you can install the MiniAV package, you will need to install a few packages. Please see the linked installation instructions before continuing.
- Python >= 3.8.2: [Further details below](#python-environments)
- [Docker](https://docker.com): [Installation instructions](https://docs.docker.com/get-docker/)
- [docker-compose](https://docs.docker.com/compose/): [Installation instructions](https://docs.docker.com/compose/install/)

Once the [prerequisites](#prerequisites) have been installed, you may proceed to installing the MiniAV package.

## Python Package 

To install the `miniav` Python package, it is fairly simple. 

### Python Environments

```{note}
This is merely a recommendation. Virtual and/or Conda environments simply isolate your Python versions and packages from other systems so that you can have different isolated environments on your system. If your main Python version is greater than 3.8.2 and you're not concerned about isolating your Python packages, ignore this section.
</div></div>
```

In order to install the `miniav` package, your Python version has to be greater than 3.8.2. This is a requirement of the [`rosbags`](https://pypi.org/project/rosbags/) package used. Different Python versions introduced and deprecated different features, which is why it is common practice to require a certain Python version.

A common and _recommended_ way of maintaining Python versions, along with their packages, on your system is through [Python Virtual Environments](https://docs.python.org/3/tutorial/venv.html) or [Anaconda](https://anaconda.org). Virtual environments isolate your Python versions and packages from other environments. Imagine you are working on a project that requires Python2.7 and another that requires Python3.8. These versions are completely incompatible with one another, so their packages and code will be, too. The solution to this problem would be to create a Python2.7 virtual environment and a Python3.8 virtual environment. The primary difference between `venv` and Anaconda is that Anaconda is not restricted to only Python packages but allows you to install other packages that use other languages. For the `miniav` package, no such non-Python packages are used, so either can be used (though Anaconda is more common). Further, `venv` requires the Python version you intend to use to be installed on your system already, which Anaconda does not.

#### Create a Python Environment with `conda`

```{note}
You will need to install Anaconda for your system before creating the environment. To do that, please refer to their [official documentation](https://docs.anaconda.com/anaconda/install/index.html).
</div></div>
```

To create a `conda` environment, you can do something like the following:

```bash
$ conda create -n miniav python=3.8.2
$ conda activate miniav
```

#### Create a Python Environment with `venv`

```{warning}
You _must_ have Python >= 3.8.2 installed already for this to work. If you don't already have Python >= 3.8.2, you will need to create an environment via [conda](#create-a-python-environment-with-conda).
</div></div>
```

To create a Python virtual environment using `venv`, you can do something like the following:

```bash
$ python -m venv miniav
```

You must then source the virtual environment. This depends on your system. See below for information on how to do that.

| Platform | Shell           | Command to activate virtual environment |
|----------|-----------------|-----------------------------------------|
| POSIX    | bash/zsh        | `$ source <venv>/bin/activate`            |
|          | fish            | `$ source <venv>/bin/activate.fish`       |
|          | csh/tcsh        | `$ source <venv>/bin/activate.csh`       |
|          | PowerShell Core | `$ <venv>/bin/Activate.ps1`              |
| Windows  | cmd.exe         | `C:\> <venv>\Scripts\activate.bat`       |
|          | PowerShell      | `PS C:\> <venv>\Scripts\Activate.ps1`    |

You may also want to refer to the [`venv` documentation](https://docs.python.org/3/library/venv.html).

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
