// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include "moveit_grasps/grasp_plan.h"

// void openGripper(trajectory_msgs::JointTrajectory& posture)
// {
//   // BEGIN_SUB_TUTORIAL open_gripper
//   /* Add both finger joints of panda robot. */
//   posture.joint_names.resize(2);
//   posture.joint_names[0] = "panda_finger_joint1";
//   posture.joint_names[1] = "panda_finger_joint2";

//   /* Set them as open, wide enough for the object to fit. */
//   posture.points.resize(1);
//   posture.points[0].positions.resize(2);
//   posture.points[0].positions[0] = 0.04;
//   posture.points[0].positions[1] = 0.04;
//   // END_SUB_TUTORIAL
// }

// void closedGripper(trajectory_msgs::JointTrajectory& posture)
// {
//   // BEGIN_SUB_TUTORIAL closed_gripper
//   /* Add both finger joints of panda robot. */
//   posture.joint_names.resize(2);
//   posture.joint_names[0] = "panda_finger_joint1";
//   posture.joint_names[1] = "panda_finger_joint2";

//   /* Set them as closed. */
//   posture.points.resize(1);
//   posture.points[0].positions.resize(2);
//   posture.points[0].positions[0] = 0.00;
//   posture.points[0].positions[1] = 0.00;
//   // END_SUB_TUTORIAL
// }

// void pick(moveit::planning_interface::MoveGroupInterface& move_group)
// {
//   // BEGIN_SUB_TUTORIAL pick1
//   // Create a vector of grasps to be attempted, currently only creating single grasp.
//   // This is essentially useful when using a grasp generator to generate and test multiple grasps.
//   std::vector<moveit_msgs::Grasp> grasps;
//   grasps.resize(1);

//   // Setting grasp pose
//   // ++++++++++++++++++++++
//   // This is the pose of panda_link8. |br|
//   // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
//   // of the cube). |br|
//   // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
//   // extra padding)
//   grasps[0].grasp_pose.header.frame_id = "panda_link0";
//   grasps[0].grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2, -M_PI / 4, -M_PI / 2);
//   grasps[0].grasp_pose.pose.position.x = 0.415;
//   grasps[0].grasp_pose.pose.position.y = 0;
//   grasps[0].grasp_pose.pose.position.z = 0.5;

//   // Setting pre-grasp approach
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
//   /* Direction is set as positive x axis */
//   grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
//   grasps[0].pre_grasp_approach.min_distance = 0.095;
//   grasps[0].pre_grasp_approach.desired_distance = 0.115;

//   // Setting post-grasp retreat
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
//   /* Direction is set as positive z axis */
//   grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
//   grasps[0].post_grasp_retreat.min_distance = 0.1;
//   grasps[0].post_grasp_retreat.desired_distance = 0.25;

//   // Setting posture of eef before grasp
//   // +++++++++++++++++++++++++++++++++++
//   openGripper(grasps[0].pre_grasp_posture);
//   // END_SUB_TUTORIAL

//   // BEGIN_SUB_TUTORIAL pick2
//   // Setting posture of eef during grasp
//   // +++++++++++++++++++++++++++++++++++
//   closedGripper(grasps[0].grasp_posture);
//   // END_SUB_TUTORIAL

//   // BEGIN_SUB_TUTORIAL pick3
//   // Set support surface as table1.
//   move_group.setSupportSurfaceName("table1");
//   // Call pick to pick up the object using the grasps given
//   move_group.pick("object", grasps);
//   // END_SUB_TUTORIAL
// }

// void place(moveit::planning_interface::MoveGroupInterface& group)
// {
//   // BEGIN_SUB_TUTORIAL place
//   // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
//   // location in
//   // verbose mode." This is a known issue and we are working on fixing it. |br|
//   // Create a vector of placings to be attempted, currently only creating single place location.
//   std::vector<moveit_msgs::PlaceLocation> place_location;
//   place_location.resize(1);

//   // Setting place location pose
//   // +++++++++++++++++++++++++++
//   place_location[0].place_pose.header.frame_id = "panda_link0";
//   place_location[0].place_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, M_PI / 2);

//   /* While placing it is the exact location of the center of the object. */
//   place_location[0].place_pose.pose.position.x = 0;
//   place_location[0].place_pose.pose.position.y = 0.5;
//   place_location[0].place_pose.pose.position.z = 0.5;

//   // Setting pre-place approach
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
//   /* Direction is set as negative z axis */
//   place_location[0].pre_place_approach.direction.vector.z = -1.0;
//   place_location[0].pre_place_approach.min_distance = 0.095;
//   place_location[0].pre_place_approach.desired_distance = 0.115;

//   // Setting post-grasp retreat
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
//   /* Direction is set as negative y axis */
//   place_location[0].post_place_retreat.direction.vector.y = -1.0;
//   place_location[0].post_place_retreat.min_distance = 0.1;
//   place_location[0].post_place_retreat.desired_distance = 0.25;

//   // Setting posture of eef after placing object
//   // +++++++++++++++++++++++++++++++++++++++++++
//   /* Similar to the pick case */
//   openGripper(place_location[0].post_place_posture);

//   // Set support surface as table2.
//   group.setSupportSurfaceName("table2");
//   // Call place to place the object using the place locations given.
//   group.place("object", place_location);
//   // END_SUB_TUTORIAL
// }

// void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
// {
//   // BEGIN_SUB_TUTORIAL table1
//   //
//   // Creating Environment
//   // ^^^^^^^^^^^^^^^^^^^^
//   // Create vector to hold 3 collision objects.
//   std::vector<moveit_msgs::CollisionObject> collision_objects;
//   collision_objects.resize(3);

//   // Add the first table where the cube will originally be kept.
//   collision_objects[0].id = "table1";
//   collision_objects[0].header.frame_id = "panda_link0";

//   /* Define the primitive and its dimensions. */
//   collision_objects[0].primitives.resize(1);
//   collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
//   collision_objects[0].primitives[0].dimensions.resize(3);
//   collision_objects[0].primitives[0].dimensions[0] = 0.2;
//   collision_objects[0].primitives[0].dimensions[1] = 0.4;
//   collision_objects[0].primitives[0].dimensions[2] = 0.4;

//   /* Define the pose of the table. */
//   collision_objects[0].primitive_poses.resize(1);
//   collision_objects[0].primitive_poses[0].position.x = 0.5;
//   collision_objects[0].primitive_poses[0].position.y = 0;
//   collision_objects[0].primitive_poses[0].position.z = 0.2;
//   // END_SUB_TUTORIAL

//   collision_objects[0].operation = collision_objects[0].ADD;

//   // BEGIN_SUB_TUTORIAL table2
//   // Add the second table where we will be placing the cube.
//   collision_objects[1].id = "table2";
//   collision_objects[1].header.frame_id = "panda_link0";

//   /* Define the primitive and its dimensions. */
//   collision_objects[1].primitives.resize(1);
//   collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
//   collision_objects[1].primitives[0].dimensions.resize(3);
//   collision_objects[1].primitives[0].dimensions[0] = 0.4;
//   collision_objects[1].primitives[0].dimensions[1] = 0.2;
//   collision_objects[1].primitives[0].dimensions[2] = 0.4;

//   /* Define the pose of the table. */
//   collision_objects[1].primitive_poses.resize(1);
//   collision_objects[1].primitive_poses[0].position.x = 0;
//   collision_objects[1].primitive_poses[0].position.y = 0.5;
//   collision_objects[1].primitive_poses[0].position.z = 0.2;
//   // END_SUB_TUTORIAL

//   collision_objects[1].operation = collision_objects[1].ADD;

//   // BEGIN_SUB_TUTORIAL object
//   // Define the object that we will be manipulating
//   collision_objects[2].header.frame_id = "panda_link0";
//   collision_objects[2].id = "object";

//   /* Define the primitive and its dimensions. */
//   collision_objects[2].primitives.resize(1);
//   collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
//   collision_objects[2].primitives[0].dimensions.resize(3);
//   collision_objects[2].primitives[0].dimensions[0] = 0.02;
//   collision_objects[2].primitives[0].dimensions[1] = 0.02;
//   collision_objects[2].primitives[0].dimensions[2] = 0.2;

//   /* Define the pose of the object. */
//   collision_objects[2].primitive_poses.resize(1);
//   collision_objects[2].primitive_poses[0].position.x = 0.5;
//   collision_objects[2].primitive_poses[0].position.y = 0;
//   collision_objects[2].primitive_poses[0].position.z = 0.5;
//   // END_SUB_TUTORIAL

//   collision_objects[2].operation = collision_objects[2].ADD;

//   planning_scene_interface.applyCollisionObjects(collision_objects);
// }

// int main(int argc, char** argv)
// {
//   ros::init(argc, argv, "panda_arm_pick_place");
//   ros::NodeHandle nh;
//   ros::AsyncSpinner spinner(1);
//   spinner.start();

//   ros::WallDuration(1.0).sleep();
//   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//   moveit::planning_interface::MoveGroupInterface group("panda_arm");
//   group.setPlanningTime(45.0);

//   addCollisionObjects(planning_scene_interface);

//   // Wait a bit for ROS things to initialize
//   ros::WallDuration(1.0).sleep();

//   group.planGraspsAndPick("object");

//   // pick(group);

//   // ros::WallDuration(1.0).sleep();

//   // place(group);

//   ros::waitForShutdown();
//   return 0;
// }



































// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// MoveIt
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// Rviz
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// Grasp
#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Baxter specific properties
#include <moveit_grasps/grasp_data.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps
{
// Table dimensions
static const double TABLE_HEIGHT = .92;
static const double TABLE_WIDTH = .85;
static const double TABLE_DEPTH = .47;
static const double TABLE_X = 0.66;
static const double TABLE_Y = 0;
static const double TABLE_Z = -0.9 / 2 + 0.01;

static const double BLOCK_SIZE = 0.04;

class GraspPlannerTest
{
public:
  // Constructor
  GraspPlannerTest() : nh_("~")
  {
    // Get arm info from param server
    const std::string parent_name = "grasp_planner_test";  // for namespacing logging messages
    
    // rosparam_shortcuts::get(parent_name, nh_, "planning_group_name", planning_group_name_);
    // rosparam_shortcuts::get(parent_name, nh_, "ee_group_name", ee_group_name_);

    ee_group_name_ = "hand";
    planning_group_name_ = "panda_arm";

    ROS_INFO_STREAM_NAMED("test", "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test", "Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load planning scene to share
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    if (planning_scene_monitor_->getPlanningScene())
    {
      planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                            "grasping_planning_scene");
      planning_scene_monitor_->getPlanningScene()->setName("grasping_planning_scene");
    }
    else
    {
      ROS_ERROR_STREAM_NAMED("test", "Planning scene not configured");
      return;
    }

    move_group_.reset(new moveit::planning_interface::MoveGroupInterface("panda_arm"));

    const robot_model::RobotModelConstPtr robot_model = planning_scene_monitor_->getRobotModel();
    // get the joint model group for the main robot arm
    arm_jmg_ = robot_model->getJointModelGroup(planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model->getModelFrame(), "/rviz_visual_tools",
                                                                   planning_scene_monitor_));

    visual_tools_->loadTrajectoryPub();
    visual_tools_->loadRobotStatePub();
    visual_tools_->loadSharedRobotState();
    visual_tools_->getSharedRobotState()->setToDefaultValues();
    visual_tools_->publishRobotState(visual_tools_->getSharedRobotState());
    robot_state::RobotStatePtr kinematic_state = visual_tools_->getSharedRobotState();

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_.reset(new GraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()));

    // ---------------------------------------------------------------------------------------------
    // Clear out old collision objects
    visual_tools_->removeAllCollisionObjects();

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_));

    // ---------------------------------------------------------------------------------------------
    // Load grasp planner
    grasp_planner_.reset(new moveit_grasps::GraspPlanner(nh_));

    // ---------------------------------------------------------------------------------------------
    // Clear Markers
    visual_tools_->deleteAllMarkers();
    Eigen::Affine3d world_cs = Eigen::Affine3d::Identity();
    visual_tools_->publishAxis(world_cs);
  }

  bool cuboidTest()
  {
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.5;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.5;
    double depth = 0.03;
    double width = 0.03;
    double height = 0.15;

    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
    visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();

    // Generate set of grasps for one object
    ROS_INFO_STREAM_NAMED("test", "Generating cuboid grasps");
    // std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                     grasp_candidates_);
    grasp_planner_->setCandidateGrasps(grasp_candidates_);
  }

  bool cylinderTest()
  {
    geometry_msgs::Pose object_pose;
    object_pose.position.x = 0.5;
    object_pose.position.y = 0.0;
    object_pose.position.z = 0.5;
    double radius = 0.015;
    double height = 0.15;

    // visual_tools_->publishCollisionCylinder(object_pose, "test_cylinder", radius, height, rviz_visual_tools::TRANSLUCENT_DARK);
    visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();

    // Generate set of grasps for one object
    ROS_INFO_STREAM_NAMED("test", "Generating cuboid grasps");
    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), radius, height, grasp_data_,
                                     grasp_candidates);
  }

  void addCollisionObjects()
  {
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = "panda_link0";
    collision_objects[2].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface_.applyCollisionObjects(collision_objects);
  }

  void planGraspsAndPick(std::string object_name)
  {
    move_group_->setSupportSurfaceName("table1");
    move_group_->setPlanningTime(45.0);
    move_group_->planGraspsAndPick(object_name);
  }

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  moveit_grasps::GraspGeneratorPtr grasp_generator_;

  // Grasp Planner
  moveit_grasps::GraspPlannerPtr grasp_planner_;

  // Grasp candidate
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_grasps::GraspDataPtr grasp_data_;

  // Shared planning scene (load once for everything)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // planning_scene_interface (make sure this works)
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  // create move group 
  moveit::planning_interface::MoveGroupInterfacePtr move_group_;
  // move_group_.setPlanningTime(45.0);

  // Arm
  const robot_model::JointModelGroup* arm_jmg_;

  // which baxter arm are we using
  std::string ee_group_name_;
  std::string planning_group_name_;

};  // end of class

}  // namespace

int main(int argc, char* argv[])
{
  int num_tests = 1;

  ros::init(argc, argv, "grasp_generation_test");

  moveit_grasps::GraspPlannerTest tester;

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // moveit::planning_interface::MoveGroupInterface group("panda_arm");
  // group.setPlanningTime(45.0);

  tester.addCollisionObjects();

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  tester.cuboidTest();
  // calling service
  std::cout << "make a call to service" << std::endl;
  tester.planGraspsAndPick("object");

  // pick(group);

  // ros::WallDuration(1.0).sleep();

  // place(group);

  ros::waitForShutdown();
  return 0;
  

  // // Seed random
  // // srand(ros::Time::now().toSec());

  // // Benchmark time
  // ros::Time start_time;
  // start_time = ros::Time::now();

  // // Run Tests
  // moveit_grasps::GraspGeneratorTest tester;
  // tester.cuboidTest();
  // // tester.cylinderTest();

  // // Benchmark time
  // double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  // ROS_INFO_STREAM_NAMED("", "Total time: " << duration);
  // std::cout << duration << "\t" << num_tests << std::endl;

  // ros::Duration(1.0).sleep();  // let rviz markers finish publishing

  // return 0;
}
