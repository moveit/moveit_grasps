/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

/* Author: Michael Lautman
 * Desc: Test suction grasp pipeline
 */

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/robot_state/conversions.h>

// MoveIt Grasp
#include <moveit_grasps/suction_grasp_generator.h>
#include <moveit_grasps/suction_grasp_filter.h>
#include <moveit_grasps/suction_grasp_data.h>
#include <moveit_grasps/grasp_planner.h>

// MoveIt Grasps
#include <moveit_visual_tools/moveit_visual_tools.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace
{
bool isStateValid(const planning_scene::PlanningScene* planning_scene,
                  const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, robot_state::RobotState* robot_state,
                  const robot_model::JointModelGroup* group, const double* ik_solution)
{
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();
  return !planning_scene->isStateColliding(*robot_state, group->getName());
}

}  // namespace

namespace moveit_grasps
{
static const std::string LOGNAME = "grasp_pipeline_demo";

class SuctionGraspPipelineTest : public ::testing::Test
{
public:
  SuctionGraspPipelineTest() : nh_("~"), verbose_(true)
  {
  }

  void setupPlanningSceneMonitor()
  {
    // Load scene
    planning_scene_monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    planning_scene_monitor_->setPlanningScenePublishingFrequency(200);
    planning_scene_monitor_->setStateUpdateFrequency(200);
    planning_scene_monitor_->getPlanningScene()->setName("grasping_planning_scene");
    planning_scene_monitor_->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                          "grasping_planning_scene");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startStateMonitor();
    ASSERT_TRUE(planning_scene_monitor_->getPlanningScene());
  }

  void setupVisualization()
  {
    // ---------------------------------------------------------------------------------------------
    // Setup moveit_visual_tools
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        robot_model_->getModelFrame(), "/rviz_visual_tools", planning_scene_monitor_);
    visual_tools_->loadMarkerPub();
    visual_tools_->loadRobotStatePub("/display_robot_state");
    visual_tools_->loadTrajectoryPub("/display_planned_path");
    visual_tools_->loadSharedRobotState();
    visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->hideRobot();
    visual_tools_->trigger();

    // Publish the global frame
    visual_tools_->publishAxis(Eigen::Isometry3d::Identity());
    visual_tools_->trigger();
  }

  void SetUp() override
  {
    std::string arm_group_name;
    std::string ee_group_name;

    ASSERT_TRUE(rosparam_shortcuts::get(LOGNAME, nh_, "arm_group_name", arm_group_name));
    ASSERT_TRUE(rosparam_shortcuts::get(LOGNAME, nh_, "ee_group_name", ee_group_name));

    robot_model_loader::RobotModelLoaderPtr robot_model_loader;
    robot_model_loader = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");

    // Load the robot model
    robot_model_ = robot_model_loader->getModel();

    ASSERT_TRUE(robot_model_ != nullptr);
    arm_jmg_ = robot_model_->getJointModelGroup(arm_group_name);
    ASSERT_TRUE(arm_jmg_ != nullptr);
    ee_jmg_ = robot_model_->getJointModelGroup(ee_group_name);
    ASSERT_TRUE(ee_jmg_ != nullptr);

    setupPlanningSceneMonitor();
    setupVisualization();
    setupGraspPipeline();
  }

  void setupGraspPipeline()
  {
    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_ =
        std::make_shared<moveit_grasps::SuctionGraspData>(nh_, ee_jmg_->getName(), visual_tools_->getRobotModel());
    ASSERT_TRUE(grasp_data_->loadGraspData(nh_, ee_jmg_->getName())) << "Failed to load Grasp Data parameters.";

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_ = std::make_shared<moveit_grasps::SuctionGraspGenerator>(visual_tools_);

    // Set the ideal grasp orientation for scoring
    std::vector<double> ideal_grasp_rpy = { 3.14, 0.0, 0.0 };
    grasp_generator_->setIdealTCPGraspPoseRPY(ideal_grasp_rpy);

    // Set custom grasp score weights
    auto grasp_score_weights = std::make_shared<moveit_grasps::SuctionGraspScoreWeights>();
    grasp_score_weights->orientation_x_score_weight_ = 2.0;
    grasp_score_weights->orientation_y_score_weight_ = 2.0;
    grasp_score_weights->orientation_z_score_weight_ = 2.0;
    grasp_score_weights->translation_x_score_weight_ = 1.0;
    grasp_score_weights->translation_y_score_weight_ = 1.0;
    grasp_score_weights->translation_z_score_weight_ = 1.0;
    // Suction gripper specific weights.
    grasp_score_weights->overhang_score_weight_ = 10.0;
    // Assign the grasp score weights in the grasp_generator
    grasp_generator_->setGraspScoreWeights(grasp_score_weights);

    // ---------------------------------------------------------------------------------------------
    // Load grasp filter
    grasp_filter_ =
        std::make_shared<moveit_grasps::SuctionGraspFilter>(visual_tools_->getSharedRobotState(), visual_tools_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp planner for approach, lift and retreat planning
    grasp_planner_ = std::make_shared<moveit_grasps::GraspPlanner>(visual_tools_);

    // -----------------------------------------------------
    // Load the motion planning pipeline
    planning_pipeline_ =
        std::make_shared<planning_pipeline::PlanningPipeline>(robot_model_, nh_, "planning_plugin", "request_adapter");
  }

  void visualizePick(const moveit_grasps::GraspCandidatePtr& grasp_candidate,
                     const moveit_msgs::MotionPlanResponse& pre_approach_plan)
  {
    EigenSTL::vector_Isometry3d waypoints;
    moveit_grasps::GraspGenerator::getGraspWaypoints(grasp_candidate, waypoints);

    // Visualize waypoints
    visual_tools_->publishAxisLabeled(waypoints[0], "pregrasp");
    visual_tools_->publishAxisLabeled(waypoints[1], "grasp");
    visual_tools_->publishAxisLabeled(waypoints[2], "lifted");
    visual_tools_->publishAxisLabeled(waypoints[3], "retreat");
    visual_tools_->trigger();

    // Hide the robot
    visual_tools_->hideRobot();
    visual_tools_->trigger();

    robot_state::RobotStatePtr current_state;
    {
      boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
          new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));
      current_state = std::make_shared<robot_state::RobotState>((*ls)->getCurrentState());
    }

    bool wait_for_animation = true;
    visual_tools_->publishTrajectoryPath(pre_approach_plan.trajectory, current_state, wait_for_animation);
    ros::Duration(0.25).sleep();
    visual_tools_->publishTrajectoryPath(grasp_candidate->segmented_cartesian_traj_[moveit_grasps::APPROACH],
                                         grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    ros::Duration(0.25).sleep();
    visual_tools_->publishTrajectoryPath(grasp_candidate->segmented_cartesian_traj_[moveit_grasps::LIFT],
                                         grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    visual_tools_->publishTrajectoryPath(grasp_candidate->segmented_cartesian_traj_[moveit_grasps::RETREAT],
                                         grasp_candidate->grasp_data_->arm_jmg_, wait_for_animation);
    ros::Duration(0.25).sleep();
  }

  bool getIKSolution(const moveit::core::JointModelGroup* arm_jmg, const Eigen::Isometry3d& target_pose,
                     robot_state::RobotState& solution, const std::string& link_name)
  {
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
        new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));

    moveit::core::GroupStateValidityCallbackFn constraint_fn = boost::bind(
        &isStateValid, static_cast<const planning_scene::PlanningSceneConstPtr&>(*ls).get(), visual_tools_, _1, _2, _3);

    // seed IK call with current state
    solution = (*ls)->getCurrentState();

    // Solve IK problem for arm
    // disable explicit restarts to guarantee close solution if one exists
    const double timeout = 0.1;
    return solution.setFromIK(arm_jmg, target_pose, link_name, timeout, constraint_fn);
  }

  bool planPreApproach(const robot_state::RobotState& goal_state, moveit_msgs::MotionPlanResponse& pre_approach_plan)
  {
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    double tolerance_above = 0.01;
    double tolerance_below = 0.01;
    // req.planner_id = "RRTConnectkConfigDefault";
    req.group_name = arm_jmg_->getName();
    req.num_planning_attempts = 5;
    req.allowed_planning_time = 1.5;
    moveit_msgs::Constraints goal =
        kinematic_constraints::constructGoalConstraints(goal_state, arm_jmg_, tolerance_below, tolerance_above);

    req.goal_constraints.push_back(goal);
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
        new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));

    // ---------------------------------
    // Change the robot current state
    // NOTE: We have to do this since Panda start configuration is in self collision.
    robot_state::RobotState rs = (*ls)->getCurrentState();
    std::vector<double> starting_joint_values = { 0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785 };
    std::vector<std::string> joint_names = { "panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4",
                                             "panda_joint5", "panda_joint6", "panda_joint7" };
    // arm_jmg_->getActiveJointModelNames();
    for (std::size_t i = 0; i < joint_names.size(); ++i)
    {
      rs.setJointPositions(joint_names[i], &starting_joint_values[i]);
    }
    rs.update();
    robot_state::robotStateToRobotStateMsg(rs, req.start_state);
    // ---------------------------

    planning_pipeline_->generatePlan(*ls, req, res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
      ROS_INFO_NAMED(LOGNAME, "Failed to plan approach successfully");
      return false;
    }

    res.getMessage(pre_approach_plan);
    return true;
  }

  bool planFullGrasp(std::vector<moveit_grasps::GraspCandidatePtr>& grasp_candidates,
                     moveit_grasps::GraspCandidatePtr& selected_grasp_candidate,
                     moveit_msgs::MotionPlanResponse& pre_approach_plan, const std::string& object_name)
  {
    moveit::core::RobotStatePtr current_state;
    {
      boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
          new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));
      current_state = std::make_shared<robot_state::RobotState>((*ls)->getCurrentState());
    }

    bool success = false;
    for (; !grasp_candidates.empty(); grasp_candidates.erase(grasp_candidates.begin()))
    {
      selected_grasp_candidate = grasp_candidates.front();
      selected_grasp_candidate->getPreGraspState(current_state);
      if (!grasp_planner_->planApproachLiftRetreat(selected_grasp_candidate, current_state, planning_scene_monitor_,
                                                   false, object_name))
      {
        ROS_INFO_NAMED(LOGNAME, "failed to plan approach lift retreat");
        continue;
      }

      robot_state::RobotStatePtr pre_grasp_state =
          selected_grasp_candidate->segmented_cartesian_traj_[moveit_grasps::APPROACH].front();
      if (!planPreApproach(*pre_grasp_state, pre_approach_plan))
      {
        ROS_INFO_NAMED(LOGNAME, "failed to plan to pregrasp_state");
        continue;
      }

      success = true;
      break;
    }
    return success;
  }

protected:
  ros::NodeHandle nh_;
  bool verbose_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  moveit_grasps::SuctionGraspGeneratorPtr grasp_generator_;
  moveit_grasps::SuctionGraspFilterPtr grasp_filter_;
  moveit_grasps::SuctionGraspDataPtr grasp_data_;
  moveit_grasps::GraspPlannerPtr grasp_planner_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const robot_model::JointModelGroup* arm_jmg_;
  const robot_model::JointModelGroup* ee_jmg_;
  robot_model::RobotModelPtr robot_model_;

};  // class SuctionGraspPipelineTest

TEST_F(SuctionGraspPipelineTest, TestGrasp)
{
  // -----------------------------------
  // Generate random object to grasp
  geometry_msgs::Pose object_pose;
  object_pose.position.x = 0.5;
  object_pose.position.y = 0.5;
  object_pose.position.z = 0.35;
  object_pose.orientation.w = -1;
  double object_x_depth = 0.05;
  double object_y_width = 0.05;
  double object_z_height = 0.025;
  std::string object_name = "target_box";

  visual_tools_->publishCollisionCuboid(object_pose, object_x_depth, object_y_width, object_z_height, object_name,
                                        rviz_visual_tools::RED);
  visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);
  visual_tools_->trigger();

  // -----------------------------------
  // Generate grasp candidates
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  ASSERT_TRUE(grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), object_x_depth, object_y_width,
                                               object_z_height, grasp_data_, grasp_candidates))
      << "Grasp generator failed to generate any valid grasps";

  // --------------------------------------------
  // Generating a seed state for filtering grasps
  robot_state::RobotStatePtr seed_state =
      std::make_shared<robot_state::RobotState>(*visual_tools_->getSharedRobotState());
  Eigen::Isometry3d eef_mount_grasp_pose =
      visual_tools_->convertPose(object_pose) * grasp_data_->tcp_to_eef_mount_.inverse();

  if (!getIKSolution(arm_jmg_, eef_mount_grasp_pose, *seed_state, grasp_data_->parent_link_->getName()))
  {
    ROS_WARN_STREAM_NAMED(LOGNAME, "The ideal seed state is not reachable. Using start state as seed.");
    boost::scoped_ptr<planning_scene_monitor::LockedPlanningSceneRW> ls(
        new planning_scene_monitor::LockedPlanningSceneRW(planning_scene_monitor_));
    seed_state = std::make_shared<robot_state::RobotState>((*ls)->getCurrentStateNonConst());
  }
  visual_tools_->publishRobotState(seed_state, rviz_visual_tools::GREEN);

  // --------------------------------------------
  // Filtering grasps
  // Note: This step also solves for the grasp and pre-grasp states and stores them in grasp candidates)
  bool filter_pregrasps = true;
  grasp_filter_->setSuctionVoxelOverlapCutoff(0.125);
  EXPECT_TRUE(grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_, seed_state,
                                          filter_pregrasps, object_name))
      << "Filter grasps failed";
  ROS_ERROR_STREAM_NAMED("test", "grasp_candidates size " << grasp_candidates.size());
  grasp_filter_->printFilterStatistics(grasp_candidates);
  ASSERT_TRUE(grasp_filter_->removeInvalidAndFilter(grasp_candidates)) << "Grasp filtering removed all grasps";
  ROS_INFO_STREAM_NAMED(LOGNAME, "" << grasp_candidates.size() << " remain after filtering");

  // Plan free-space approach, cartesian approach, lift and retreat trajectories
  moveit_grasps::GraspCandidatePtr selected_grasp_candidate;
  moveit_msgs::MotionPlanResponse pre_approach_plan;

  ASSERT_TRUE(planFullGrasp(grasp_candidates, selected_grasp_candidate, pre_approach_plan, object_name))
      << "Failed to plan grasp motions";

  visualizePick(selected_grasp_candidate, pre_approach_plan);
}

}  // namespace moveit_grasps

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "suction_grasp_pipeline_test");

  // run test
  int result = RUN_ALL_TESTS();

  ros::shutdown();
  return result;
}
