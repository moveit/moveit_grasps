# MoveIt Grasps

A basic grasp generator for objects such as blocks or cylinders for use with the MoveIt pick and place pipeline. Does not consider friction cones or other dynamics. It also has support for suction grippers.

Its current implementation takes as input a pose vector (postition and orientation) and generates a large number of potential grasp approaches and directions. Also includes a grasp filter for removing kinematically infeasible grasps via threaded IK solvers.

This package includes:

 - Pose-based grasp generator for a block
 - Separate grasp generators for custom objects such as rectanguar or cylindrical objects
 - Grasp filter
 - Demo code and visualizations

<img src="https://picknik.ai/assets/images/logo.jpg" width="100">

Developed by Dave Coleman, Andy McEvoy, and Mike Lautman at [PickNik Consulting](http://picknik.ai/) with many contributors.

[![Build Status](https://travis-ci.org/ros-planning/moveit_grasps.svg?branch=melodic-devel)](https://travis-ci.org/ros-planning/moveit_grasps)

<img src="https://raw.githubusercontent.com/ros-planning/moveit_grasps/melodic-devel/resources/demo.png" />

## Install

### Ubuntu Debian

> Note: this package has not been released yet

```
sudo apt-get install ros-melodic-moveit-grasps
```

### Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies.

Melodic (Ubuntu 18.04):

1. Re-use or create a catkin workspace:

        export CATKIN_WS=~/ws_catkin/
        mkdir -p $CATKIN_WS/src
        cd $CATKIN_WS/src

1. Download the required repositories and install any dependencies:

        git clone git@github.com:ros-planning/moveit_grasps.git
        wstool init .
        wstool merge moveit_grasps/moveit_grasps.rosinstall
        wstool update
        rosdep install --from-paths . --ignore-src --rosdistro melodic

1. Configure and build the workspace:

        cd $CATKIN_WS
        catkin config --extend /opt/ros/melodic --cmake-args -DCMAKE_BUILD_TYPE=Release
        catkin build

1. Source the workspace.

        source devel/setup.bash

## Usage Instructions

For detailed usage instructions visit the MoveIt Grasps [tutorial](https://ros-planning.github.io/moveit_tutorials/doc/moveit_grasps/moveit_grasps_tutorial.html).

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin lint -W2 --rosdistro melodic

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run tests.

    catkin run_tests --no-deps --this -i
