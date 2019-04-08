# MoveIt! Grasps

A basic grasp generator for objects such as blocks or cylinders for use with the MoveIt! pick and place pipeline. Does not consider friction cones or other dynamics. It also has support for suction grippers.

Its current implementation takes as input a pose vector (postition and orientation) and generates a large number of potential grasp approaches and directions. Also includes a grasp filter for removing kinematically infeasible grasps via threaded IK solvers.

This package includes:

 - Pose-based grasp generator for a block
 - Separate grasp generators for custom objects such as rectanguar or cylindrical objects
 - Grasp filter
 - Demo code and visualizations

<img src="https://picknik.ai/images/logo.jpg" width="100">

Developed by Dave Coleman, Andy McEvoy, and Mike Lautman at [PickNik Consulting](http://picknik.ai/) with many contributors.

[![Build Status](https://travis-ci.org/PickNikRobotics/moveit_grasps.svg?branch=melodic-devel)](https://travis-ci.org/PickNikRobotics/moveit_grasps)

<img src="https://raw.githubusercontent.com/PickNikRobotics/moveit_grasps/melodic-devel/resources/demo.png" />

## Install

### Ubuntu Debian

> Note: this package has not been released yet

```
sudo apt-get install ros-melodic-moveit-grasps
```

### Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. Depending on your current version of ROS, use:

Melodic:
```
rosdep install --from-paths src --ignore-src --rosdistro melodic
```

## Robot-Agnostic Configuration

You will first need a configuration file that described your robot's end effector geometry. Currently an example format can be seen in this repository at [config_robot/baxter_grasp_data.yaml](https://github.com/PickNikRobotics/moveit_grasps/blob/melodic-devel/config_robot/baxter_grasp_data.yaml). See the comments within that file for explanations.

To load that file at launch, you copy the example in the file [launch/grasp_test.launch](https://github.com/PickNikRobotics/moveit_grasps/blob/melodic-devel/launch/load_panda.launch) where you should see the line ``<rosparam command="load" file="$(find moveit_grasps)/config_robot/panda_grasp_data.yaml"/>``.

Within that file you will find all of the gripper specific parameters necessary for customizing MoveIt! Grasps with any suction or finger gripper

These values can be visualized by launching `grasp_generator_demo.launch`, `grasp_poses_visualizer_demo.launch`, and `grasp_pipeline_demo.launch`.
The result should look like the following:

![Grasp Poses Visualization](https://raw.githubusercontent.com/PickNikRobotics/moveit_grasps/melodic-devel/resources/moveit_grasps_poses.jpeg)

### Some Important Parameters:

#### grasp_pose_to_eef_transform

The `grasp_pose_to_eef_transform` represents the transform from the wrist to the end-effector. This parameter is provided to allow different URDF end effectors to all work together without recompiling code. In MoveIt! the EE always has a parent link, typically the wrist link or palm link. That parent link should have its Z-axis pointing towards the object you want to grasp i.e. where your pointer finger is pointing. This is the convention laid out in "Robotics" by John Craig in 1955. However, a lot of URDFs do not follow this convention, so this transform allows you to fix it.

Additionally, the x-axis should be pointing up along the grasped object, i.e. the circular axis of a (beer) bottle if you were holding it. The y-axis should be point towards one of the fingers.

#### Switch from Bin to Shelf Picking with ``setIdealGraspPoseRPY`` and ``setIdealGraspPose``

The ``setIdealGraspPoseRPY`` and ``setIdealGraspPose`` methods in GraspGenerator can be used to select an ideal grasp orientation for picking. These methods is used to score grasp candidates favoring grasps that are closer to the desired orientation. This is useful in applications such as bin and shelf picking where you would want to pick the objects from a bin with a grasp that is vertically alligned and you would want to pick obejects from a shelf with a grasp that is horozontally alligned.

## Demo Scripts

There are four demo scripts in this package. To view the tests, first start Rviz with:

    roslaunch moveit_grasps rviz.launch

To see the entire MoveIt! Grasps pipeline in actoin:

    roslaunch moveit_grasps grasp_pipeline_demo.launch

To visualize gripper specific parameters:

    roslaunch moveit_grasps grasp_poses_visualizer_demo.launch

To test just grasp generation for randomly placed blocks:

    roslaunch moveit_grasps demo_grasp_generator.launch

To test the grasp filtering:

    roslaunch moveit_grasps demo_filter.launch

### Grasp Filter

When filtered, the colors represent the following:

    RED - grasp filtered by ik
    PINK - grasp filtered by collision
    MAGENTA - grasp filtered by cutting plane
    YELLOW - grasp filtered by orientation
    BLUE - pregrasp filtered by ik
    CYAN - pregrasp filtered by collision
    GREEN - valid

## Tested Robots

 - UR5
 - Jaco2
 - [Baxter](https://github.com/davetcoleman/baxter_cpp)
 - [REEM](http://wiki.ros.org/Robots/REEM)
 - Panda

## Example Code

The most current example for using MoveIt! Grasps is the `grasp_pipeline_demo` which can be found [here](https://github.com/PickNikRobotics//moveit_grasps/melodic-devel/src/grasp_pipeline_demo.cpp).

There are other example implementations:

 - [baxter_pick_place](https://github.com/davetcoleman/baxter_cpp/tree/melodic-devel/baxter_pick_place)
 - [reem_tabletop_grasping](https://github.com/pal-robotics/reem_tabletop_grasping)

## Testing and Linting

To run [roslint](http://wiki.ros.org/roslint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin build --no-status --no-deps --this --make-args roslint

To run [catkin lint](https://pypi.python.org/pypi/catkin_lint), use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/).

    catkin lint -W2 --rosdistro melodic

Use the following command with [catkin-tools](https://catkin-tools.readthedocs.org/) to run tests.

    catkin run_tests --no-deps --this -i
