/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, PickNik LLC
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

/* Author: Dave Coleman
 */

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Grasp generation
#include <moveit_grasps/two_finger_grasp_generator.h>

// Grasp Data
#include <moveit_grasps/two_finger_grasp_data.h>

namespace moveit_grasps
{
const std::size_t NUM_EXPECTED_GRASPS = 672;

class TwoFingerGraspGeneratorTest : public ::testing::Test
{
public:
  TwoFingerGraspGeneratorTest() : nh_("~"), verbose_(true), ee_group_name_("hand")
  {
  }
  void SetUp() override
  {
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("panda_link0");
    grasp_data_ =
        std::make_shared<moveit_grasps::TwoFingerGraspData>(nh_, ee_group_name_, visual_tools_->getRobotModel());
    ASSERT_TRUE(grasp_data_->loadGraspData(nh_, ee_group_name_));
  }

protected:
  ros::NodeHandle nh_;
  bool verbose_;
  std::string ee_group_name_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  moveit_grasps::TwoFingerGraspDataPtr grasp_data_;

};  // class TwoFingerGraspGeneratorTest

TEST_F(TwoFingerGraspGeneratorTest, ConstructDestruct)
{
  TwoFingerGraspGenerator grasp_generator(visual_tools_, verbose_);
}

TEST_F(TwoFingerGraspGeneratorTest, TwoFingerGraspCandidateConfig)
{
  // Default constructor
  TwoFingerGraspCandidateConfig grasp_candidate_config;
  EXPECT_TRUE(grasp_candidate_config.enable_corner_grasps_);

  // Grasp Types
  grasp_candidate_config.enableAllGraspTypes();
  EXPECT_TRUE(grasp_candidate_config.enable_corner_grasps_);
  EXPECT_TRUE(grasp_candidate_config.enable_face_grasps_);
  EXPECT_TRUE(grasp_candidate_config.enable_variable_angle_grasps_);
  EXPECT_TRUE(grasp_candidate_config.enable_edge_grasps_);

  // Grasp Types
  grasp_candidate_config.disableAllGraspTypes();
  EXPECT_FALSE(grasp_candidate_config.enable_corner_grasps_);
  EXPECT_FALSE(grasp_candidate_config.enable_face_grasps_);
  EXPECT_FALSE(grasp_candidate_config.enable_variable_angle_grasps_);
  EXPECT_FALSE(grasp_candidate_config.enable_edge_grasps_);

  // Grasp Axes
  grasp_candidate_config.disableAllGraspAxes();
  EXPECT_FALSE(grasp_candidate_config.generate_x_axis_grasps_);
  EXPECT_FALSE(grasp_candidate_config.generate_y_axis_grasps_);
  EXPECT_FALSE(grasp_candidate_config.generate_z_axis_grasps_);

  // Grasp Axes
  grasp_candidate_config.enableAllGraspAxes();
  EXPECT_TRUE(grasp_candidate_config.generate_x_axis_grasps_);
  EXPECT_TRUE(grasp_candidate_config.generate_y_axis_grasps_);
  EXPECT_TRUE(grasp_candidate_config.generate_z_axis_grasps_);

  grasp_candidate_config.disableAll();
  EXPECT_FALSE(grasp_candidate_config.generate_x_axis_grasps_);
  EXPECT_FALSE(grasp_candidate_config.generate_y_axis_grasps_);
  EXPECT_FALSE(grasp_candidate_config.generate_z_axis_grasps_);
  EXPECT_FALSE(grasp_candidate_config.enable_corner_grasps_);
  EXPECT_FALSE(grasp_candidate_config.enable_face_grasps_);
  EXPECT_FALSE(grasp_candidate_config.enable_variable_angle_grasps_);
  EXPECT_FALSE(grasp_candidate_config.enable_edge_grasps_);
}

TEST_F(TwoFingerGraspGeneratorTest, GenerateFaceGrasps)
{
  TwoFingerGraspCandidateConfig grasp_candidate_config;
  grasp_candidate_config.enable_face_grasps_ = true;

  // Construct
  TwoFingerGraspGenerator grasp_generator(visual_tools_, verbose_);

  // Input
  Eigen::Isometry3d cuboid_pose = Eigen::Isometry3d::Identity();
  cuboid_pose.translation().x() = 1;
  cuboid_pose.translation().y() = 2;
  cuboid_pose.translation().z() = 3;
  double depth = 0.01;
  double width = 0.01;
  double height = 0.01;

  // Generate X Axis
  std::vector<GraspCandidatePtr> grasp_candidates;
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, X_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());

  // Generate Y Axis
  grasp_candidates.clear();
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Y_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());

  // Generate Z Axis
  grasp_candidates.clear();
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Z_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());

  // Check contents of a grasp candidate
  GraspCandidatePtr grasp = grasp_candidates.front();

  // Grasp Msg
  EXPECT_EQ(grasp->grasp_.id, "Grasp448");
  EXPECT_GT(grasp->grasp_.grasp_pose.pose.position.x, 0);
  EXPECT_GT(grasp->grasp_.grasp_pose.pose.position.y, 0);
  EXPECT_GT(grasp->grasp_.grasp_pose.pose.position.z, 0);

  // Grasp Data
  EXPECT_NE(nullptr, grasp->grasp_data_);

  // Original Pose
  EXPECT_EQ(1, grasp->cuboid_pose_.translation().x());
  EXPECT_EQ(2, grasp->cuboid_pose_.translation().y());
  EXPECT_EQ(3, grasp->cuboid_pose_.translation().z());

  // No filtering has happend yet
  EXPECT_FALSE(grasp->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_IK);
  EXPECT_FALSE(grasp->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_CUTTING_PLANE);
  EXPECT_FALSE(grasp->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_ORIENTATION);
  EXPECT_FALSE(grasp->grasp_filtered_code_ == GraspFilterCode::GRASP_FILTERED_BY_IK_CLOSED);
  EXPECT_FALSE(grasp->grasp_filtered_code_ == GraspFilterCode::PREGRASP_FILTERED_BY_IK);

  // No IK solutions have been generated yet
  EXPECT_TRUE(grasp->grasp_ik_solution_.empty());
  EXPECT_TRUE(grasp->pregrasp_ik_solution_.empty());

  // No planning has occured yet
  EXPECT_TRUE(grasp->segmented_cartesian_traj_.empty());
}

TEST_F(TwoFingerGraspGeneratorTest, GenerateEdgeGrasps)
{
  TwoFingerGraspCandidateConfig grasp_candidate_config;
  grasp_candidate_config.enable_edge_grasps_ = true;

  // Construct
  TwoFingerGraspGenerator grasp_generator(visual_tools_, verbose_);

  // Input
  Eigen::Isometry3d cuboid_pose = Eigen::Isometry3d::Identity();
  cuboid_pose.translation().x() = 1;
  cuboid_pose.translation().y() = 2;
  cuboid_pose.translation().z() = 3;
  double depth = 0.01;
  double width = 0.01;
  double height = 0.01;

  // Generate X Axis
  std::vector<GraspCandidatePtr> grasp_candidates;
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, X_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());

  // Generate Y Axis
  grasp_candidates.clear();
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Y_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());

  // Generate Z Axis
  grasp_candidates.clear();
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Z_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());
}

TEST_F(TwoFingerGraspGeneratorTest, GenerateCornerGrasps)
{
  TwoFingerGraspCandidateConfig grasp_candidate_config;
  grasp_candidate_config.enable_corner_grasps_ = true;

  // Construct
  TwoFingerGraspGenerator grasp_generator(visual_tools_, verbose_);

  // Input
  Eigen::Isometry3d cuboid_pose = Eigen::Isometry3d::Identity();
  cuboid_pose.translation().x() = 1;
  cuboid_pose.translation().y() = 2;
  cuboid_pose.translation().z() = 3;
  double depth = 0.01;
  double width = 0.01;
  double height = 0.01;

  // Generate X Axis
  std::vector<GraspCandidatePtr> grasp_candidates;
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, X_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());

  // Generate Y Axis
  grasp_candidates.clear();
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Y_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());

  // Generate Z Axis
  grasp_candidates.clear();
  grasp_generator.generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Z_AXIS, grasp_data_,
                                           grasp_candidate_config, grasp_candidates);

  EXPECT_EQ(NUM_EXPECTED_GRASPS, grasp_candidates.size());
}

// TODO(davetcoleman): Test all helper functions
// TODO(davetcoleman): Test addGrasp
// TODO(davetcoleman): Test scoreGrasp

}  // namespace moveit_grasps

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "grasp_generator_test");

  // ros::AsyncSpinner spinner(1);
  // spinner.start();

  int result = RUN_ALL_TESTS();

  // spinner.stop();
  ros::shutdown();
  return result;
}
