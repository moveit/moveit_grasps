/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman
   Desc:   Tests the grasp generator filter
*/

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

class GraspGeneratorTest
{
public:
  // Constructor
  GraspGeneratorTest() : nh_("~")
  {
    // Get arm info from param server
    const std::string parent_name = "grasp_filter_test";  // for namespacing logging messages
    rosparam_shortcuts::get(parent_name, nh_, "planning_group_name", planning_group_name_);
    rosparam_shortcuts::get(parent_name, nh_, "ee_group_name", ee_group_name_);

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

    const robot_model::RobotModelConstPtr robot_model = planning_scene_monitor_->getRobotModel();
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
    robot_state::RobotStatePtr robot_state = visual_tools_->getSharedRobotState();

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
    // Load grasp filter
    grasp_filter_.reset(new moveit_grasps::GraspFilter(robot_state, visual_tools_));

    // ---------------------------------------------------------------------------------------------
    // Clear Markers
    visual_tools_->deleteAllMarkers();
    Eigen::Affine3d world_cs = Eigen::Affine3d::Identity();
    visual_tools_->publishAxis(world_cs);
  }

  bool testRandomGrasps(std::size_t num_tests)
  {
    // Generate grasps for a bunch of random objects

    // Loop
    for (std::size_t i = 0; i < num_tests; ++i)
    {
      if (!ros::ok())
        break;

      ROS_INFO_STREAM_NAMED("test", "Adding random object " << i + 1 << " of " << num_tests);

      // Generate random cuboid
      geometry_msgs::Pose object_pose;
      double depth;
      double width;
      double height;
      rviz_visual_tools::RandomPoseBounds pose_bounds(0.4, 0.6, -0.25, 0.25, 0,
                                                      0.5);  // xmin, xmax, ymin, ymax, zmin, zmax
      visual_tools_->generateRandomCuboid(object_pose, depth, width, height, pose_bounds);
      visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
      visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);

      // Generate set of grasps for one object
      ROS_INFO_STREAM_NAMED("test", "Generating cuboid grasps");
      std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
      grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                       grasp_candidates);

      // add grasps at variable depth
      // grasp_generator_->addVariableDepthGrasps(visual_tools_->convertPose(object_pose), grasp_data_,
      // grasp_candidates);

      // Filter the grasp for only the ones that are reachable
      ROS_INFO_STREAM_NAMED("test", "Filtering grasps kinematically");
      bool filter_pregrasps = true;
      bool verbose = false;  // note: setting this to true will disable threading
      // int direction = 1;

      // world X goes into shelf, so filter all grasps behind the YZ oriented plane of the object
      // Eigen::Affine3d filter_pose = Eigen::Affine3d::Identity();
      // filter_pose.translation() = visual_tools_->convertPose(object_pose).translation();
      // //visual_tools_->publishAxis(filter_pose);
      // grasp_filter_->clearCuttingPlanes();
      // grasp_filter_->addCuttingPlane(filter_pose, moveit_grasps::YZ, direction);

      // // can only reach the object from the front
      // filter_pose = Eigen::Affine3d::Identity();
      // filter_pose = filter_pose * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
      // filter_pose.translation() = visual_tools_->convertPose(object_pose).translation();
      // //visual_tools_->publishAxis(filter_pose);
      // grasp_filter_->clearDesiredGraspOrientations();
      // grasp_filter_->addDesiredGraspOrientation(filter_pose, M_PI / 4.0);

      std::size_t valid_grasps = grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_,
                                                             visual_tools_->getSharedRobotState(), filter_pregrasps);

      if (valid_grasps == 0)
      {
        ROS_ERROR_STREAM_NAMED("test", "No valid grasps found after IK filtering");
        continue;
      }

      ROS_INFO_STREAM_NAMED("test", "finished trial, wating 5s to start next trial");
      ros::Duration(5.0).sleep();  // give some time to look at results of each trial
    }                              // for each trial
    return true;
  }

  bool unitTest()
  {
    // Generate random cuboid
    geometry_msgs::Pose object_pose;
    generateTestObject(object_pose);
    double depth = 0.1;
    double width = 0.1;
    double height = 0.1;
    visual_tools_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
    visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);

    // Generate set of grasps for one object
    ROS_INFO_STREAM_NAMED("test", "Generating cuboid grasps");
    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                     grasp_candidates);

    // add grasps at variable depth
    // grasp_generator_->addVariableDepthGrasps(visual_tools_->convertPose(object_pose), grasp_data_, grasp_candidates);

    // Filter the grasp for only the ones that are reachable
    ROS_INFO_STREAM_NAMED("test", "Filtering grasps kinematically");
    bool filter_pregrasps = true;
    bool verbose = false;  // note: setting this to true will disable threading
    int direction = 1;

    // world X goes into shelf, so filter all grasps behind the YZ oriented plane of the object
    Eigen::Affine3d filter_pose = Eigen::Affine3d::Identity();
    filter_pose.translation() = visual_tools_->convertPose(object_pose).translation();
    // visual_tools_->publishAxis(filter_pose);
    grasp_filter_->clearCuttingPlanes();
    grasp_filter_->addCuttingPlane(filter_pose, moveit_grasps::YZ, direction);

    // can only reach the object from the front
    filter_pose = Eigen::Affine3d::Identity();
    filter_pose = filter_pose * Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitY());
    filter_pose.translation() = visual_tools_->convertPose(object_pose).translation();
    // visual_tools_->publishAxis(filter_pose);
    grasp_filter_->clearDesiredGraspOrientations();
    grasp_filter_->addDesiredGraspOrientation(filter_pose, M_PI / 4.0);

    std::size_t valid_grasps = grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_,
                                                           visual_tools_->getSharedRobotState(), filter_pregrasps);

    if (valid_grasps == 0)
    {
      ROS_ERROR_STREAM_NAMED("test", "No valid grasps found after IK filtering");
      return false;
    }
    return true;
  }

  void generateTestObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = 0.75;
    object_pose.position.y = 0.1;
    object_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();
  }

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  moveit_grasps::GraspGeneratorPtr grasp_generator_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Grasp filter
  moveit_grasps::GraspFilterPtr grasp_filter_;

  // data for generating grasps
  moveit_grasps::GraspDataPtr grasp_data_;

  // Shared planning scene (load once for everything)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

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

  ros::init(argc, argv, "grasp_generator_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  moveit_grasps::GraspGeneratorTest tester;
  tester.testRandomGrasps(num_tests);
  // tester.unitTest();

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("", "Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep();  // let rviz markers finish publishing

  return 0;
}
