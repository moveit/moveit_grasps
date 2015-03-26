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
#include <moveit_grasps/grasps.h>
#include <moveit_grasps/grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Baxter specific properties
#include <moveit_grasps/grasp_data.h>

namespace moveit_grasps
{

// Table dimensions
static const double TABLE_HEIGHT = .92;
static const double TABLE_WIDTH = .85;
static const double TABLE_DEPTH = .47;
static const double TABLE_X = 0.66;
static const double TABLE_Y = 0;
static const double TABLE_Z = -0.9/2+0.01;

static const double BLOCK_SIZE = 0.04;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  moveit_grasps::GraspsPtr grasp_generator_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Grasp filter
  moveit_grasps::GraspFilterPtr grasp_filter_;

  // data for generating grasps
  moveit_grasps::GraspData grasp_data_;

  // Shared planning scene (load once for everything)
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // which baxter arm are we using
  std::string arm_;
  std::string ee_group_name_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspGeneratorTest(int num_tests) 
    : nh_("~")
  {
    // Get arm info from param server
    nh_.param("arm", arm_, std::string("left"));
    nh_.param("ee_group_name", ee_group_name_, std::string(arm_ + "_hand"));
    planning_group_name_ = arm_ + "_arm";

    ROS_INFO_STREAM_NAMED("test","Arm: " << arm_);
    ROS_INFO_STREAM_NAMED("test","End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test","Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load planning scene to share
    planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));    
    const robot_model::RobotModelConstPtr robot_model = planning_scene_monitor_->getRobotModel();
    const robot_model::JointModelGroup* arm_jmg = robot_model->getJointModelGroup(planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools(robot_model->getModelFrame(), "/end_effector_marker", 
                                                                   planning_scene_monitor_));
    visual_tools_->setFloorToBaseHeight(-0.9);
    visual_tools_->loadTrajectoryPub();
    visual_tools_->loadRobotStatePub();
    visual_tools_->loadSharedRobotState();
    visual_tools_->getSharedRobotState()->setToDefaultValues();
    robot_state::RobotStatePtr robot_state = visual_tools_->getSharedRobotState();

    // ---------------------------------------------------------------------------------------------
    // Load grasp data
    if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()))
      ros::shutdown();

    // ---------------------------------------------------------------------------------------------
    // Clear out old collision objects
    visual_tools_->removeAllCollisionObjects();

    // Create a collision table for fun
    visual_tools_->publishCollisionTable(TABLE_X, TABLE_Y, 0, TABLE_WIDTH, TABLE_HEIGHT, TABLE_DEPTH, "table");

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_.reset( new moveit_grasps::Grasps(visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    grasp_filter_.reset(new moveit_grasps::GraspFilter(robot_state, visual_tools_) );

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    std::vector<GraspCandidatePtr> candidate_grasps;

    const moveit::core::JointModelGroup* ee_jmg = robot_state->getRobotModel()->getJointModelGroup(grasp_data_.ee_group_name_);
  
    // Loop
    for (int i = 0; i < num_tests; ++i)
    {
      if(!ros::ok())
        break;

      // Clear markers
      visual_tools_->deleteAllMarkers();

      ROS_INFO_STREAM_NAMED("test","Adding random object " << i+1 << " of " << num_tests);

      // Generate random cuboid
      geometry_msgs::Pose object_pose;
      double depth;
      double width;
      double height;
      rviz_visual_tools::RandomPoseBounds pose_bounds(0.2, 0.5, 0, 0.5, 0, 0.5); // xmin, xmax, ymin, ymax, zmin, zmax
      visual_tools_->generateRandomCuboid(object_pose, depth, width, height, pose_bounds);
      visual_tools_->publishCuboid(object_pose, depth, width, height);

      // Generate set of grasps for one object
      ROS_INFO_STREAM_NAMED("test","Generating cuboid grasps");
      std::vector<moveit_msgs::Grasp> possible_grasps;
      double max_grasp_size = 0.10; // TODO: verify max object size that can be grasped
      grasp_generator_->generateCuboidGrasps( visual_tools_->convertPose(object_pose), depth, width, height, max_grasp_size,
                                              grasp_data_, possible_grasps);

      // Convert to the correct type for filtering
      candidate_grasps = grasp_filter_->convertToGraspCandidatePtrs(possible_grasps);

      // Filter the grasp for only the ones that are reachable
      ROS_INFO_STREAM_NAMED("test","Filtering grasps kinematically");
      bool filter_pregrasps = true;
      bool verbose = true;
      bool verbose_if_failed = true;
      std::size_t valid_grasps = grasp_filter_->filterGraspsKinematically(candidate_grasps, arm_jmg, filter_pregrasps, verbose, verbose_if_failed);

      if (valid_grasps == 0)
      {
        ROS_ERROR_STREAM_NAMED("test","No valid grasps found after IK filtering");
        continue;
      }

      // Convert the candidate_grasps into a format moveit_visual_tools can use
      std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions;
      for (std::size_t i = 0; i < candidate_grasps.size(); ++i)
      {
        // Check if valid grasp
        if (!candidate_grasps[i]->validated_by_ik_) // || !candidate_grasps[i]->validated_by_collision_)
          continue; // not valid

        trajectory_msgs::JointTrajectoryPoint new_point;
        new_point.positions = candidate_grasps[i]->grasp_ik_solution_;
        ik_solutions.push_back(new_point);
      }
      double animation_speed = 0.1;
      visual_tools_->publishIKSolutions(ik_solutions, planning_group_name_, animation_speed);
    } // for each trial


  }

  void generateTestObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = 0.5;
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

  void generateRandomObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = visual_tools_->dRand(0.7,TABLE_DEPTH);
    object_pose.position.y = visual_tools_->dRand(-TABLE_WIDTH/2,-0.1);
    object_pose.position.z = TABLE_Z + TABLE_HEIGHT / 2.0 + BLOCK_SIZE / 2.0;
  
    // Orientation
    double angle = M_PI * visual_tools_->dRand(0.1,1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();
  }

}; // end of class

} // namespace


int main(int argc, char *argv[])
{
  int num_tests = 10;

  ros::init(argc, argv, "grasp_generator_test");

  // Allow the action server to recieve and send ros messages
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Check for verbose flag
  bool verbose = false;
  if (argc > 1)
  {
    for (std::size_t i = 0; i < argc; ++i)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (much slower)");
        verbose = true;
      }
    }
  }
  
  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  moveit_grasps::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("","Total time: " << duration);
  std::cout << duration << "\t" << num_tests << std::endl;

  ros::Duration(1.0).sleep(); // let rviz markers finish publishing

  return 0;
}
