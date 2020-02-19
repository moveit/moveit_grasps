/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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

/* Author: Dale Koenig
 * Desc: Unit tests for suction grasps
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

}  // end anonymous namespace

namespace moveit_grasps
{
static const std::string LOGNAME = "suction_grasp_unit_tests";

class SuctionGraspUnitTests : public ::testing::Test
{
public:
  SuctionGraspUnitTests() : nh_("~"), verbose_(true)
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

    robot_model_loader_ = std::make_shared<robot_model_loader::RobotModelLoader>("robot_description");

    // Load the robot model
    robot_model_ = robot_model_loader_->getModel();

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

    voxel_size_x_ = grasp_data_->suction_voxel_matrix_->getVoxelWidthX();
    voxel_size_y_ = grasp_data_->suction_voxel_matrix_->getVoxelWidthY();

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

  robot_state::RobotStatePtr generateSeedState(const Eigen::Isometry3d& object_pose)
  {
    // --------------------------------------------
    // Generating a seed state for filtering grasps
    robot_state::RobotStatePtr seed_state =
        std::make_shared<robot_state::RobotState>(*visual_tools_->getSharedRobotState());
    Eigen::Isometry3d eef_mount_grasp_pose = object_pose * grasp_data_->tcp_to_eef_mount_.inverse();
    visual_tools_->publishAxis(eef_mount_grasp_pose, rviz_visual_tools::MEDIUM);

    if (!getIKSolution(arm_jmg_, eef_mount_grasp_pose, *seed_state, grasp_data_->parent_link_->getName()))
    {
      ROS_WARN_STREAM_NAMED("generateSeedState", "The ideal seed state is not reachable. Using start state as seed.");
      planning_scene_monitor::LockedPlanningSceneRO ls(planning_scene_monitor_);
      seed_state = std::make_shared<robot_state::RobotState>(ls->getCurrentState());
    }
    visual_tools_->publishRobotState(seed_state, rviz_visual_tools::GREEN);
    visual_tools_->trigger();
    return seed_state;
  }

protected:
  ros::NodeHandle nh_;
  bool verbose_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  moveit_grasps::SuctionGraspGeneratorPtr grasp_generator_;
  moveit_grasps::SuctionGraspFilterPtr grasp_filter_;
  moveit_grasps::SuctionGraspDataPtr grasp_data_;
  moveit_grasps::GraspPlannerPtr grasp_planner_;

  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  const robot_model::JointModelGroup* arm_jmg_;
  const robot_model::JointModelGroup* ee_jmg_;
  robot_model::RobotModelPtr robot_model_;

  double voxel_size_x_;
  double voxel_size_y_;

};  // class SuctionGraspUnitTests

// Test that all grasps are filtered for a single box colliding with the robot
TEST_F(SuctionGraspUnitTests, TestGraspFilter)
{
  // -----------------------------------
  auto object_pose = Eigen::Isometry3d(Eigen::Translation3d(0.0, 0.0, 0.0));
  Eigen::Vector3d object_size(1.7 * voxel_size_x_, 1.7 * voxel_size_y_, .05);
  std::string object_name = "target_box";

  visual_tools_->publishCollisionCuboid(object_pose, object_size, object_name, rviz_visual_tools::BLUE);
  visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);

  // -----------------------------------
  // Generate grasp candidates
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  ASSERT_TRUE(grasp_generator_->generateGrasps(object_pose, object_size.x(), object_size.y(), object_size.z(),
                                               grasp_data_, grasp_candidates))
      << "Grasp generator failed to generate any valid grasps";

  // --------------------------------------------
  // Filtering grasps
  bool filter_pregrasps = false;
  grasp_filter_->setSuctionVoxelOverlapCutoff(0.9);
  EXPECT_TRUE(grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_,
                                          generateSeedState(object_pose), filter_pregrasps, object_name))
      << "Filter grasps failed";
  ASSERT_GT(grasp_candidates.size(), 0u);
  ASSERT_FALSE(grasp_filter_->removeInvalidAndFilter(grasp_candidates)) << "Expected grasp filter to remove all grasps";
}

// Ensure that we filter all grasps when trying to pick up an object smaller than a suction voxel and surrounded on all
// sides
TEST_F(SuctionGraspUnitTests, TestFilterSuctionIK)
{
  // -----------------------------------
  auto object_pose = Eigen::Isometry3d(Eigen::Translation3d(0.3, 0.3, 0.25));
  Eigen::Vector3d object_size(0.7 * voxel_size_x_, 0.7 * voxel_size_y_, .05);
  std::string object_name = "target_box";

  visual_tools_->publishCollisionCuboid(object_pose, object_size, object_name, rviz_visual_tools::BLUE);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(object_size.x(), 0.0, 0.0), object_size,
                                        "object1", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(-object_size.x(), 0.0, 0.0), object_size,
                                        "object2", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(0.0, object_size.y(), 0.0), object_size,
                                        "object3", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(0.0, -object_size.y(), 0.0), object_size,
                                        "object4", rviz_visual_tools::RED);
  visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);

  // -----------------------------------
  // Generate grasp candidates
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  ASSERT_TRUE(grasp_generator_->generateGrasps(object_pose, object_size.x(), object_size.y(), object_size.z(),
                                               grasp_data_, grasp_candidates))
      << "Grasp generator failed to generate any valid grasps";

  // --------------------------------------------
  // Filtering grasps
  bool filter_pregrasps = false;
  grasp_filter_->setSuctionVoxelOverlapCutoff(0.9);
  EXPECT_TRUE(grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_,
                                          generateSeedState(object_pose), filter_pregrasps, object_name))
      << "Filter grasps failed";
  ASSERT_GT(grasp_candidates.size(), 0u);
  ASSERT_FALSE(grasp_filter_->removeInvalidAndFilter(grasp_candidates)) << "Expected grasp filter to remove all grasps";
}

// Ensure that we do not filter all grasps when trying to pick up an object larger than a suction voxel and surrounded
// on all sides
TEST_F(SuctionGraspUnitTests, TestFilterSuctionIKHighOverlap)
{
  // -----------------------------------
  auto object_pose = Eigen::Isometry3d(Eigen::Translation3d(0.3, 0.3, 0.25));
  Eigen::Vector3d object_size(1.7 * voxel_size_x_, 1.7 * voxel_size_y_, .05);
  std::string object_name = "target_box";

  visual_tools_->publishCollisionCuboid(object_pose, object_size, object_name, rviz_visual_tools::BLUE);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(object_size.x(), 0.0, 0.0), object_size,
                                        "object1", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(-object_size.x(), 0.0, 0.0), object_size,
                                        "object2", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(0.0, object_size.y(), 0.0), object_size,
                                        "object3", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(0.0, -object_size.y(), 0.0), object_size,
                                        "object4", rviz_visual_tools::RED);
  visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);

  // -----------------------------------
  // Generate grasp candidates
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  ASSERT_TRUE(grasp_generator_->generateGrasps(object_pose, object_size.x(), object_size.y(), object_size.z(),
                                               grasp_data_, grasp_candidates))
      << "Grasp generator failed to generate any valid grasps";

  // --------------------------------------------
  // Filtering grasps
  bool filter_pregrasps = false;
  grasp_filter_->setSuctionVoxelOverlapCutoff(0.9);
  EXPECT_TRUE(grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_,
                                          generateSeedState(object_pose), filter_pregrasps, object_name))
      << "Filter grasps failed";
  ASSERT_GT(grasp_candidates.size(), 0u);
  ASSERT_TRUE(grasp_filter_->removeInvalidAndFilter(grasp_candidates)) << "Grasp filtering removed all grasps";
}

// Ensure that we do not filter all grasps when trying to pick up an object larger than a suction voxel and surrounded
// on all sides
TEST_F(SuctionGraspUnitTests, DISABLED_TestFilterSuctionIKLowOverlap)
{
  // This test is identical to TestFilterSuctionIKHighOverlap except with a lower overlap cutoff
  // -----------------------------------
  auto object_pose = Eigen::Isometry3d(Eigen::Translation3d(0.3, 0.3, 0.25));
  Eigen::Vector3d object_size(1.7 * voxel_size_x_, 1.7 * voxel_size_y_, .05);
  std::string object_name = "target_box";

  visual_tools_->publishCollisionCuboid(object_pose, object_size, object_name, rviz_visual_tools::BLUE);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(object_size.x(), 0.0, 0.0), object_size,
                                        "object1", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(-object_size.x(), 0.0, 0.0), object_size,
                                        "object2", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(0.0, object_size.y(), 0.0), object_size,
                                        "object3", rviz_visual_tools::RED);
  visual_tools_->publishCollisionCuboid(object_pose * Eigen::Translation3d(0.0, -object_size.y(), 0.0), object_size,
                                        "object4", rviz_visual_tools::RED);
  visual_tools_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);

  // -----------------------------------
  // Generate grasp candidates
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates;
  ASSERT_TRUE(grasp_generator_->generateGrasps(object_pose, object_size.x(), object_size.y(), object_size.z(),
                                               grasp_data_, grasp_candidates))
      << "Grasp generator failed to generate any valid grasps";

  // --------------------------------------------
  // Filtering grasps
  bool filter_pregrasps = false;
  grasp_filter_->setSuctionVoxelOverlapCutoff(0.125);
  EXPECT_TRUE(grasp_filter_->filterGrasps(grasp_candidates, planning_scene_monitor_, arm_jmg_,
                                          generateSeedState(object_pose), filter_pregrasps, object_name))
      << "Filter grasps failed";
  ASSERT_GT(grasp_candidates.size(), 0u);
  ASSERT_TRUE(grasp_filter_->removeInvalidAndFilter(grasp_candidates)) << "Grasp filtering removed all grasps";
}

}  // namespace moveit_grasps

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "suction_grasp_unit_tests");

  // run test
  int result = RUN_ALL_TESTS();

  ros::shutdown();
  return result;
}
