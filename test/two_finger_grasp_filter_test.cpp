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

/* Author: Henning Kayser
 * Desc: Test filtering grasp candidates
 */

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Grasp
#include <moveit_grasps/two_finger_grasp_generator.h>
#include <moveit_grasps/two_finger_grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/two_finger_grasp_data.h>

namespace moveit_grasps
{
class GraspFilterTest : public ::testing::Test
{
public:
  GraspFilterTest() : nh_("~"), verbose_(true), ee_group_name_("hand")
  {
  }

  void SetUp() override
  {
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor =
        std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
    if (planning_scene_monitor->getPlanningScene())
    {
      planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
                                                           "grasping_planning_scene");
      planning_scene_monitor->getPlanningScene()->setName("grasping_planning_scene");
    }
    const robot_model::RobotModelConstPtr robot_model = planning_scene_monitor->getRobotModel();
    arm_jmg_ = robot_model->getJointModelGroup("panda_arm");
    ASSERT_TRUE(arm_jmg_ != nullptr);
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        robot_model->getModelFrame(), "/rviz_visual_tools", planning_scene_monitor);
    grasp_generator_ = std::make_shared<TwoFingerGraspGenerator>(visual_tools_);
    grasp_filter_ = std::make_shared<TwoFingerGraspFilter>(visual_tools_->getSharedRobotState(), visual_tools_);
    grasp_data_ = std::make_shared<TwoFingerGraspData>(nh_, ee_group_name_, visual_tools_->getRobotModel());

    ASSERT_TRUE(grasp_data_->loadGraspData(nh_, ee_group_name_));
  }

protected:
  ros::NodeHandle nh_;
  bool verbose_;
  std::string ee_group_name_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  TwoFingerGraspGeneratorPtr grasp_generator_;
  TwoFingerGraspFilterPtr grasp_filter_;
  TwoFingerGraspDataPtr grasp_data_;
  const robot_model::JointModelGroup* arm_jmg_;
};  // class GraspFilterTest

TEST_F(GraspFilterTest, TestGraspFilter)
{
  // parameters for random cuboid pose and dimensions sampling
  const double cuboid_size_min = 0.01;
  const double cuboid_size_max = 0.0125;
  const double xmin = 0.5;
  const double xmax = 0.7;
  const double ymin = -0.25;
  const double ymax = 0.25;
  const double zmin = 0.2;
  const double zmax = 0.7;

  // Generate grasps for a bunch of random objects
  const std::size_t num_tests = 5;
  for (std::size_t i = 0; i < num_tests; ++i)
  {
    // Generate random cuboid
    rviz_visual_tools::RandomPoseBounds pose_bounds(xmin, xmax, ymin, ymax, zmin, zmax);
    rviz_visual_tools::RandomCuboidBounds cuboid_bounds(cuboid_size_min, cuboid_size_max);

    // push cuboid to planning scene
    geometry_msgs::Pose object_pose;
    double depth, width, height;
    visual_tools_->generateRandomCuboid(object_pose, depth, width, height, pose_bounds, cuboid_bounds);

    // Generate set of grasps for one object
    std::vector<GraspCandidatePtr> grasp_candidates;

    // Configure the desired types of grasps
    TwoFingerGraspCandidateConfig grasp_generator_config = TwoFingerGraspCandidateConfig();
    grasp_generator_config.disableAll();
    grasp_generator_config.enable_face_grasps_ = true;
    grasp_generator_config.generate_y_axis_grasps_ = true;
    grasp_generator_config.generate_x_axis_grasps_ = true;
    grasp_generator_config.generate_z_axis_grasps_ = true;

    // generate grasps
    grasp_generator_->setGraspCandidateConfig(grasp_generator_config);
    grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                     grasp_candidates);

    // Filter the grasp for only the ones that are reachable
    bool filter_pregrasps = true;
    bool success = grasp_filter_->filterGrasps(grasp_candidates, visual_tools_->getPlanningSceneMonitor(), arm_jmg_,
                                               visual_tools_->getSharedRobotState(), filter_pregrasps);

    EXPECT_TRUE(success) << "Checks if filterGrasps (without object in the planning scene) ran without issue";

    std::size_t remaining_grasps = grasp_filter_->removeInvalidAndFilter(grasp_candidates);
    EXPECT_NE(remaining_grasps, 0u) << "No valid grasps remain after filtering";

    // add the target box to the ps
    std::string object_name = "target_box";
    visual_tools_->publishCollisionCuboid(object_pose, depth, width, height, object_name, rviz_visual_tools::RED);

    // Filter the grasp for only the ones that are reachable
    success = grasp_filter_->filterGrasps(grasp_candidates, visual_tools_->getPlanningSceneMonitor(), arm_jmg_,
                                          visual_tools_->getSharedRobotState(), filter_pregrasps, object_name);
    EXPECT_TRUE(success) << "Checks if filterGrasps (with object in the planning scene and ACM setting) ran without "
                            "issue";

    remaining_grasps = grasp_filter_->removeInvalidAndFilter(grasp_candidates);
    std::size_t expected_remaining_grasps = 0;
    EXPECT_NE(remaining_grasps, expected_remaining_grasps) << "No valid grasps remain after filtering with collision "
                                                              "object and ACM settings";

    success = grasp_filter_->filterGrasps(grasp_candidates, visual_tools_->getPlanningSceneMonitor(), arm_jmg_,
                                          visual_tools_->getSharedRobotState(), filter_pregrasps);
    EXPECT_TRUE(success) << "Checks if filterGrasps (with object in the planning scene and NO ACM setting) ran without "
                            "issue";

    remaining_grasps = grasp_filter_->removeInvalidAndFilter(grasp_candidates);
    expected_remaining_grasps = 0;
    EXPECT_EQ(remaining_grasps, expected_remaining_grasps) << "Valid grasps found after IK filtering despite "
                                                              "collisions";
  }
}

TEST_F(GraspFilterTest, TestGraspFilterCompareScores)
{
  moveit_msgs::Grasp grasp;
  GraspDataPtr grasp_data;
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  grasp.grasp_quality = 2;
  GraspCandidatePtr grasp_a = std::make_shared<GraspCandidate>(grasp, grasp_data, pose);
  grasp.grasp_quality = 1;
  GraspCandidatePtr grasp_b = std::make_shared<GraspCandidate>(grasp, grasp_data, pose);
  EXPECT_TRUE(TwoFingerGraspFilter::compareGraspScores(grasp_a, grasp_b));
}

}  // namespace moveit_grasps

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "two_finger_grasp_filter_test");

  // run test
  int result = RUN_ALL_TESTS();

  ros::shutdown();
  return result;
}
