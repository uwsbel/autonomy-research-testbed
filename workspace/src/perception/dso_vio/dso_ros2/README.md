# ROS2 Wrapper for DSO SLAM Library

# Dependencies
* [DSO SLAM library](https://github.com/JakobEngel/dso) Change CMakeFile, path of the DSO Library.
* OpenCV

# Installation
  * Clone repository to your workspace source folder `git clone https://github.com/goktug97/dso_ros2`
  *  Run `colcon build` in workspace root.
  *  Source your workspace

# Usage
You can change image topic that wrapper uses,
Currently it subcribed to topic that camera
publishes. See dso/src/.
For example you can use simulation camera image topic to 
use DSO in your gazebo simulation.

I was just testing to see if I can make it work with ROS2 so the
library is not polished. You need calibration folder with
calibration.txt pcalib.txt vignette.png and you need to run the commands
in the folder that contains calibration folder.

To run with camera use 
	
`ros2 launch dso_bringup dso_bringup.launch.py`

To run standalone use

`ros2 run dso dso_ros`

You can also use [my fork](https://github.com/goktug97/dso) of DSO for save option
