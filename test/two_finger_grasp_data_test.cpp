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

   Test loading the Panda config
*/

// C++
#include <string>

// ROS
#include <ros/ros.h>

// Testing
#include <gtest/gtest.h>

// Grasp generation
#include <moveit_grasps/two_finger_grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

namespace moveit_grasps
{
class TwoFingerGraspDataTest : public ::testing::Test
{
public:
  TwoFingerGraspDataTest() : nh_("~"), ee_group_name_("hand")
  {
  }

  void SetUp() override
  {
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("panda_link0");
    grasp_data_ = std::make_shared<TwoFingerGraspData>(nh_, ee_group_name_, visual_tools_->getRobotModel());
    ASSERT_TRUE(grasp_data_->loadGraspData(nh_, ee_group_name_));
  }

protected:
  ros::NodeHandle nh_;
  std::string ee_group_name_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  TwoFingerGraspDataPtr grasp_data_;
};  // class GraspGenerator

TEST_F(TwoFingerGraspDataTest, CheckConfigValues)
{
  // Grasp Pose To EEF Pose
  EXPECT_EQ(grasp_data_->tcp_to_eef_mount_.translation().x(), 0);
  double tolerance = 0.000001;
  EXPECT_LT(grasp_data_->tcp_to_eef_mount_.translation().y() - (0), tolerance);
  EXPECT_LT(grasp_data_->tcp_to_eef_mount_.translation().z() - (-0.105), tolerance);

  // Pre Grasp Posture
  EXPECT_EQ(grasp_data_->pre_grasp_posture_.header.frame_id, "world");
  EXPECT_GT(grasp_data_->pre_grasp_posture_.header.stamp.toSec(), 0);
  EXPECT_EQ(grasp_data_->pre_grasp_posture_.points.size(), (size_t)1);
  EXPECT_GT(grasp_data_->pre_grasp_posture_.points[0].positions.size(), (size_t)0);

  // Grasp Posture
  EXPECT_EQ(grasp_data_->grasp_posture_.header.frame_id, "world");
  EXPECT_GT(grasp_data_->grasp_posture_.header.stamp.toSec(), 0);
  EXPECT_EQ(grasp_data_->grasp_posture_.points.size(), (size_t)1);
  EXPECT_GT(grasp_data_->grasp_posture_.points[0].positions.size(), (size_t)0);

  // Semantics
  EXPECT_EQ(grasp_data_->base_link_, "world");
  EXPECT_EQ(grasp_data_->ee_jmg_->getName(), "hand");
  // TODO (mlautman-2/13/19): restore this test once https://github.com/ros-planning/panda_moveit_config/pull/20 is
  // released
  // EXPECT_EQ(grasp_data_->arm_jmg_->getName(), "panda_arm");
  EXPECT_EQ(grasp_data_->parent_link_->getName(), "panda_link8");
  EXPECT_EQ(grasp_data_->robot_model_->getName(), "panda");

  // Geometry doubles
  EXPECT_GT(grasp_data_->angle_resolution_, 0);
  EXPECT_GT(grasp_data_->grasp_max_depth_, 0);
  EXPECT_GT(grasp_data_->grasp_resolution_, 0);
  EXPECT_GT(grasp_data_->grasp_depth_resolution_, 0);
  EXPECT_GT(grasp_data_->grasp_min_depth_, 0);
  EXPECT_GT(grasp_data_->gripper_finger_width_, 0);
  EXPECT_GT(grasp_data_->max_grasp_width_, 0);
  EXPECT_GT(grasp_data_->approach_distance_desired_, 0);
  EXPECT_GT(grasp_data_->retreat_distance_desired_, 0);
  EXPECT_GT(grasp_data_->lift_distance_desired_, 0);
  EXPECT_GT(grasp_data_->grasp_padding_on_approach_, 0);
  EXPECT_GT(grasp_data_->max_finger_width_, 0);
  EXPECT_GT(grasp_data_->min_finger_width_, 0);
}

TEST_F(TwoFingerGraspDataTest, SetRobotState)
{
  moveit::core::RobotStatePtr robot_state = visual_tools_->getSharedRobotState();

  // Pre Grasp
  grasp_data_->setRobotStatePreGrasp(robot_state);
  EXPECT_EQ(grasp_data_->pre_grasp_posture_.points[0].positions[0],
            robot_state->getJointPositions("panda_finger_joint1")[0]);

  // Grasp
  grasp_data_->setRobotStateGrasp(robot_state);
  EXPECT_EQ(grasp_data_->grasp_posture_.points[0].positions[0],
            robot_state->getJointPositions("panda_finger_joint1")[0]);
}

TEST_F(TwoFingerGraspDataTest, fingerWidthToGraspPosture)
{
  moveit::core::RobotStatePtr robot_state = visual_tools_->getSharedRobotState();

  // Pre Grasp
  grasp_data_->setRobotStatePreGrasp(robot_state);
  EXPECT_EQ(grasp_data_->pre_grasp_posture_.points[0].positions[0],
            robot_state->getJointPositions("panda_finger_joint1")[0]);

  // Grasp
  grasp_data_->setRobotStateGrasp(robot_state);
  EXPECT_EQ(grasp_data_->grasp_posture_.points[0].positions[0],
            robot_state->getJointPositions("panda_finger_joint1")[0]);
}

// TODO(davetcoleman): write test for remainder of this class

}  // namespace moveit_grasps

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "two_finger_grasp_data_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();

  int result = RUN_ALL_TESTS();

  spinner.stop();
  ros::shutdown();
  return result;
}
