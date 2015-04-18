MoveIt! Grasps
====================

A basic grasp generator for objects such as blocks or cylinders for use with the MoveIt! pick and place pipeline. Does not consider friction cones or other dynamics. 

Its current implementation takes as input a pose vector (postition and orientation) and generates a large number of potential grasp approaches and directions. Also includes a grasp filter for removing kinematically infeasible grasps via threaded IK solvers.

This package includes:

 - pose-based grasp generator for a block
 - Separate grasp generators for custom objects such as rectanguar or cylindrical objects
 - Grasp filter
 - Test code and visualizations

Developed by [Dave Coleman](http://dav.ee) at the Correll Robotics Lab, University of Colorado Boulder with outside contributors.

<img align="right" src="https://raw.github.com/davetcoleman/moveit_grasps/hydro-devel/resources/demo.png" /> 

## Install

### Ubuntu Debian

Indigo:
```
sudo apt-get install ros-indigo-moveit-grasps
```

### Install From Source

Clone this repository into a catkin workspace, then use the rosdep install tool to automatically download its dependencies. Depending on your current version of ROS, use:

Indigo:
```
rosdep install --from-paths src --ignore-src --rosdistro indigo
```

## Robot-Agnostic Configuration

You will first need a configuration file that described your robot's end effector geometry. Currently an example format can be seen in this repository at [config_robot/baxter_grasp_data.yaml](https://github.com/davetcoleman/moveit_grasps/blob/indigo-devel/config_robot/baxter_grasp_data.yaml). See the comments within that file for explanations. 

To load that file at launch, you copy the example in the file [launch/grasp_test.launch](https://github.com/davetcoleman/moveit_grasps/blob/indigo-devel/launch/grasp_test.launch) where you should see the line ``<rosparam command="load" file="$(find moveit_grasps)/config_robot/baxter_grasp_data.yaml"/>``.

Within that file you can specify the following (example taken from jaco):

    # ee group name as defined in the MoveIt! SRDF
    end_effector_name: 'gripper'  

    # actuated joints in end effector
    joints : ['jaco_joint_finger_1','jaco_joint_finger_2','jaco_joint_finger_3']

    # open position (pre-grasp)
    pregrasp_posture : [0.697, 0.697, 0.697]

    # time to wait before grasping
    pregrasp_time_from_start : 4.0

    # close position (grasp)
    grasp_posture : [0.0, 0.0, 0.0]

    # time to wait after grasping
    grasp_time_from_start : 4.0

    # desired pose from end effector to grasp - [x,y,z]
    grasp_pose_to_eef_translation :  [-0.05, 0, 0]

    # desired pose from end effector to grasp - [roll, pitch, yall], in standard 3,2,1 notation
    grasp_pose_to_eef_rotation : [1.5707, 0, 0]

    # max depth of fingers - distance from finger tip to inner palm
    finger_to_palm_depth : 0.11

These values can be visualized by launching `grasp_test_rviz.launch` and `grasp_pose_visualizer.launch`.
The result should look like the following:
![Grasp Poses Visualization](https://bytebucket.org/cuamazonchallenge/moveit_grasps/raw/6a0397bce5a309428e0fa25366e0d58015c93433/resources/moveit_grasps_poses.jpeg?token=ca124fc4d9cd8f2678a7392b3a568f7883f8ee3b)

Poses Visualized: Object, Grasp, EE
Distances: `finger_to_palm_depth`, `pre(post)grasp_distance`, `pre(post)grasp_min_distance`

### grasp_pose_to_eef

The ``grasp_pose_to_eef`` translation is to allow different URDF end effectors to all work the same. In MoveIt! the EE always has a parent link, typically the wrist link or palm link. That parent link should have its Z-axis pointing towards the object you want to grasp i.e. where your pointer finger is pointing. This is the convention laid out in "Robotics" by John Craig in 1955. However, a lot of URDFs do not follow this convention, so this transform allows you to fix it.

Additionally, the x-axis should be pointing up along the grasped object, i.e. the circular axis of a (beer) bottle if you were holding it. The y-axis should be point towards one of the fingers.

## Testing

There are two tests scripts in this package. To view the tests, first start Rviz with:

    roslaunch moveit_grasps rviz.launch

To test just grasp generation for randomly placed blocks:

    roslaunch moveit_grasps test_grasp_generator.launch 

To also test the grasp filtering:

    roslaunch moveit_grasps test_filter.launch baxter:=true

### Grasp Filter

When filtered, the colors represent the following:

    RED - grasp filtered by ik
    PINK - grasp filtered by collision
    MAGENTA - grasp filtered by cutting plane
    YELLOW - grasp filtered by orientation
    BLUE - pregrasp filtered by ik
    CYAN - pregrasp filtered by collision
    GREEN - valid

### Bounding Box From Mesh

    roslaunch moveit_grasps rviz.launch
    roslaunch moveit_grasps test_bounding_box.launch

Each mesh in the products folder will be displayed with the calculated bounding box. Hit `enter` to move to the next mesh.

## Tested Robots

 - [Baxter](https://github.com/davetcoleman/baxter_cpp)
 - [REEM](http://wiki.ros.org/Robots/REEM)

## Example Code

A new (still in development) example tool is ``moveit_blocks.h`` located in the ``include`` folder. It gives you a complete pick and place pipeline using this package and MoveIt, and all you need is the appropriate config file and launch file. An example launch file can be found [here](https://github.com/davetcoleman/clam/blob/master/clam_pick_place/launch/pick_place.launch).

There are currently example implementations:

 - [baxter_pick_place](https://github.com/davetcoleman/baxter_cpp/tree/indigo-devel/baxter_pick_place)
 - [reem_tabletop_grasping](https://github.com/pal-robotics/reem_tabletop_grasping)

## TODO

Features we'd like to see added to this project:

 - Make grasp quality metric better informed
 - Integrate into Setup Assistant GUI

## Contributors

 - Dave Coleman, CU Boulder @davetcoleman
 - Andy McEvoy, CU Boulder @mcevoyandy
 - Bence Magyar, PAL Robotics @bmagyar