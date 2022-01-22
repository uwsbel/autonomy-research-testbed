# Background

The MiniAV platform is purpose built to be an all-in-one test platform for autonomous algorithm development and simulation validation. Provided is documented hardware, a pre-made control stack, an optimized development-to-deployment workflow, and a database system to easily store and interact with ROS-based data files.

In this guide, general background of the project is discussed and an overview of design decisions is provided. 

## Purpose

As mentioned before, the MiniAV platform is an all-in-one test platform for autonomous vehicles. Providing this platform aims to leverage the high-fidelity simulation engine [Chrono](https://projectchrono.org) as the test method prior to deploying the code on the real car. Furthermore, the development workflow and data processing pipeline is meant to be generic and scalable to other hardware beyond the MiniAV. Utilizing [ROS 2](https://docs.ros.org/en/galactic/index.html) for the control stack, the sensing support is built on packages from the ROS community and will continue to grow as ROS 2 is further adapted of the first iteration of ROS.

There are also many other existing platforms that provide similar functionality; [DonkeyCar](https://www.donkeycar.com/), [PARV](https://digital.wpi.edu/concern/student_works/st74ct36z?locale=en), [PiCar](https://www.instructables.com/PiCar-an-Autonomous-Car-Platform/), [AutoRally](https://autorally.github.io/), and [MIT RACECAR](https://mit-racecar.github.io/) to name a few. Unlike the aforementioned vehicle platforms, MiniAV is built much larger from a [1:6th scale chassis from Redcat Racing](https://www.redcatracing.com/products/shredder?variant=31050562797658). This allows larger and more sensors to be added to the vehicle chassis. Modular 3D printed part files are provided that can be configured in multiple ways to mount sensor suites for different applications. Voltage rails are also provided to power sensors with varying power usage.

This simulation environment, built on top of Chrono, utilizes multiple modules to create a virtual world for autonomy development. The vehicle module, [Chrono::Vehicle](https://api.projectchrono.org/group__vehicle.html), implements high-fidelity vehicle models, for which a MiniAV model is provided. The Chrono::Vehicle module also has advanced terrain models that can be used to represent hard, concret floors to Soil Contact Model (SCM) based deformable terrain; the latter useful for modeling off-road scenarios. Then [Chrono::Sensor](https://api.projectchrono.org/group__sensor.html) simulates sensors that can be used to replicate the data produced by the physical counterparts.

## Hardware

```{todo}
To write...
```

## Control Stack

```{todo}
To write...
```

## Development Environment

The development environment is built on top of [Docker](https://www.docker.com). A tutorial for how to use the development workflow can be found [here](./tutorials/using_the_development_environment.md).

## Database

```{todo}
To write...
```
