# Vehicle Computers

This page describes how we organize the vehicle computers. **Please** review this document fully before making any changes to the vehicle computers.

## Overview

Each vehicle has one computer which runs the vehicle service which is responsible for all vehicle related tasks. This includes:
- Reading sensor data
- Controlling actuators
- Running autonomy algorithms

### Folder Structure

```
~/
└── sbel/
    └── autonomy-research-testbed/
```

### Branches

Each vehicle should have it's own branch. The branch name should be the name of the vehicle. For example, the branch for the `art-5` vehicle is `art-5`.

## Setup

This section outlines the general setup for each vehicle computer. Each vehicle computer should follow this setup so new users can have a consistent experience across vehicles.

### Run the setup script

A setup script was written to automate the setup process. To run the script, run the following command:

```bash
wget -O - https://raw.githubusercontent.com/uwsbel/autonomy-research-testbed/tree/master/vehicles/setup.sh | bash

```

> [!NOTE]
> The script will prompt you for your password. This is required to install the necessary packages. You should review the script to ensure it is safe to run (it obviously should be, but it's good practice).

This script does the following:
- Installs the necessary packages
- Set the jetson power and fan modes (if it's a jetson)
- Sets up docker permissions
- Installs miniconda
- Adds some configurations to the `.bashrc` file

### Checkout the vehicle branch

Next, we need to create the branch for the vehicle we are working on. For example, if we are working on the `art-5` vehicle, we would run:

```bash
$ git checkout -b art-5
```

### Pushing to the remote repository

Pushing from one of these computers is somewhat complicated. The computer is shared between people, but there is a single user. Therefore, when we push to the remote repository, we need to make sure we attach the correct github user to the commit. Fortunately, in the [`bashrc`](../../vehicles/bashrc) script, there are some helper functions to make this easier.

First, you need to create a ssh key. To do this, run the following:

```bash
$ sbel-ssh-keygen
```

This will prompt you for your NetID (to save the ssh key to), your name and email (for github), and a passphrase (so others can't push as you). All arguments are required.

Next, you need to add the ssh key to your github account. To do this, run the following:

```bash
$ cat ~/.ssh/id_rsa_<your_net_id>.pub
```

That output should be copied to your github ssh keys page.

> [!NOTE]
> The above commands (`sbel-ssh-keygen` and adding the key to github) need only be run once. The following you must run every time before you commit.

Finally, you need to set the git user. To do this, run the following:

```bash
$ sbel-ssh-add
```

You will be prompted for your NetID. This command will add your ssh key to the ssh agent. It will also set the name and email to git. Both are temporary and will be reset when you exit the shell.
