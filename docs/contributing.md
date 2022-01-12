# Contributing

Contributing to the repository is fairly easy. The CLI was developed to be scalable and provide easy flexibiliity when creating new commands. There could be bugs or additional subcommands that would like to be added, so please see below for instructions on how to actually make contributions to this project.

> Note: If you're contributing to the repository, it can be assumed you know what you're doing. Please be thoughtful in your changes and consult the correct people if you're looking to make changes.

## Setup

There are two forms of contributions: source code or documentation. Editing the documentation is as simple as cloning the repo and adding/editing content within the `docs` folder. All documentation is written in `markdown` and converted to `html` through `myst_parser` and `sphinx`. To edit the source code, as well as the documentation, you will want to install the package through a symlink.

> **Note**: A `conda` or `virtualenv` will add isolation to your python environments and reduce conflicts amongst packages. It is _highly_ recommended to use one!!

### Cloning the Repo

Clone the repo as normal:

```bash
git clone https://github.com/WisconsinAutonomous/wa_cli.git && cd wa_cli
```

### Installing a Symbolic Linked Version for Testing

A symbolic link or symlink is a file that references another. The advantages of symlinks is that a folder or file can _essentially_ be placed in two separate locations. In reference to this repository, we want to create a symlinked install because when we edit the code within the cloned repo, we want that change also to be reflected in the installed files.

From within your `wa_cli` directory, we can have `setuptools` do this for us with the following command (run from within the `wa_cli` root directory):

```bash
python setup.py develop
```

You should now be able to edit the source and have those changes be reflected in whatever file imports `wa_cli`!

### Deploy your Changes

[GitHub actions](https://github.com/features/actions) are used to automatically build the site and [GitHub pages](https://pages.github.com/) are used to host the static site. To update deployed content, you have to push to the `develop` branch and create a pull request on the `master` branch. Once the pull request is merged the site will rebuild with your changes. Please ensure there are no errors in your code/documentation before doing so, as you may get an email from github if something bad happens.

Further, to update the package available on [PyPI](https://pypi.org/project/wa-cli/), you must create a [git tag](https://git-scm.com/book/en/v2/Git-Basics-Tagging). When a tag is created and pushed to GitHub, it will start an Action which will automatically pushed the new release to PyPI. See [versioning](#versioning) for information on how versioning works with `wa_cli`. The Github Action only runs when the tag is pushed to master through a merge request. To create a tag, you may do the following:

```bash
git tag v3.0.1
git push origin <branch> --tags
```

#### Versioning

Versioning is done automatically through `tags` by [setuptools\_scm](https://github.com/pypa/setuptools_scm). When a tag is pushed to the `master` branch, a new package is pushed to PyPI with the attached tag. Therefore, you must ensure the tag you push is *after* the previous tags seen on GitHub.

## Guidelines

A lot of work has gone into making this package functional and scalable. Please consider all of the following guidelines and follow them to ensure the repository will persist for a long time.

### File Structure

The simulator is structured as follows:
```
wa_cli
├── LICENSE
├── docs/				# Contains documentation
├── wa_cli/			# Source code
└── setup.py		# Package description and installation instructions for pip
```

### Editing the Source Code

If you plan on editing the source code, please visit the `wa_cli/` folder. The `wa_cli/` folder is structured as follows:
```
wa_cli/
├── utils/			# Utility files for use by the rest of the package
│   └── ...
├── scripts/		# Scripts used for the 'wa script' subcommand
│   └── ...        	
└── ...					# Core wa_cli code
```

As stated earlier, unless given approval by the managers of the repository, there should be no need to edit the source code. The Object Oriented nature of the package means you can just inherit the base classes and add your own logic _outside_ the repo (no need to edit the source). However, bugs or nice features may be added. See [this section](#installing-a-symbolic-linked-version-for-testing) to install the repo for development purposes.

#### Commenting 

Commenting your code is not only _required_ when contributing to this repository, but also common practice in almost every place where code is written.

Please follow [Google's guidelines for Python Styling](https://google.github.io/styleguide/pyguide.html). These comments are also used to automatically generate the documentation. For Visual Studio Code users, the [Python Docstring Generator](https://github.com/NilsJPWerner/autoDocstring) package may be helpful.

### Editing the Documentation

If you plan on editing the documentation pages (i.e. adding a tutorial or fixing an existing page), please visit the `docs/` folder. The `docs/` folder is structured as follows:
```
docs/
├── _static/						# Static files that persist through the build process
│   ├── css/custom.css  # Custom css changes that are different from the default furo theme
│   └── ...        			# Images, favicons, etc.
├── usage.md						# Usage reference guide for the wa_cli
├── installation.md			# Installation build instructions
├── contributing.md			# Contributing tab with instructions on how to contribute to the repo
├── conf.py							# Settings related to extensions, themes, etc.
└── index.md						# The "home" page
```

Please try to maintain the file structure as described above. All tabs with only a single page (i.e. background or contributing), should have their `markdown` file with in the `docs/` folder. If the tab has or will have multiple pages (i.e. a tutorials tab), create a folder titled the same as that tab. To add pages, insert the name of the file without the `.md` extension within the table of contents inside `index.md`. Each folder should also contain an `index.md` used as the home page of that tab.

Markdown files are converted to reStructuredText by `myst_parser` which is used by the documentation package [Sphinx](https://www.sphinx-doc.org/en/master/). Both Markdown and reStructuredText have their advantages and disadvantages, `myst_parser` allows us to use the easy to understand `markdown` language but also compile the content down to something Sphinx understands. To see additional features of the `myst_parser` library, please visit their [website](https://myst-parser.readthedocs.io/en/latest/).

## Building the Documentation

There are multiple ways to build sphinx documentation. The easiest is using the `Makefile` or `make.bat` file provided directly in this repository. You will need to install all the necessary dependencies and build the html pages. To do that, run the following commands:
```bash
cd wa_cli/docs
pip install -r requirements.txt
make html
```

To view the build, go to your browser, and open the `index.html` file located inside `docs/build/html/`.

`sphinx-autobuild` is also extremely easy to use and will automatically build the html pages when a change is made. See their [PyPI page](https://pypi.org/project/sphinx-autobuild/).
