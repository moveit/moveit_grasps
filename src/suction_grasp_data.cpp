/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder, PAL Robotics, S.L.
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
 *   * Neither the name of Univ of CO, Boulder, PAL Robotics, S.L.
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
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
 */

/* Authors: Dave Coleman, Bence Magyar
   Description: Data class used by the grasp generator.
*/

#include <moveit_grasps/suction_grasp_data.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// C++
#include <cmath>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Pose conversion
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace moveit_grasps
{
const std::string LOGNAME = "grasp_data.suction_gripper";

SuctionGraspData::SuctionGraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                                   const moveit::core::RobotModelConstPtr& robot_model)
  : GraspData(nh, end_effector, robot_model)
{
}

bool SuctionGraspData::loadGraspData(const ros::NodeHandle& nh, const std::string& end_effector)
{
  if (!GraspData::loadGraspData(nh, end_effector))
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "GraspData::loadGraspData failed");
    return false;
  }
  // Overwrite postures since they are not used with suction grippers
  pre_grasp_posture_.joint_names = {};
  pre_grasp_posture_.points.resize(1);
  pre_grasp_posture_.points[0].positions = {};

  grasp_posture_.joint_names = {};
  grasp_posture_.points.resize(1);
  grasp_posture_.points[0].positions = {};

  // Load all other parameters
  const std::string parent_name = "grasp_data";  // for namespacing logging messages
  std::size_t error = 0;

  // Search within the sub-namespace of this end effector name
  ros::NodeHandle child_nh(nh, end_effector);

  int suction_rows_count, suction_cols_count;
  double active_suction_range_x, active_suction_range_y;
  error += !rosparam_shortcuts::get(parent_name, child_nh, "active_suction_range_x", active_suction_range_x);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "active_suction_range_y", active_suction_range_y);
  child_nh.param<int>("suction_rows_count", suction_rows_count, 1);
  child_nh.param<int>("suction_cols_count", suction_cols_count, 1);
  suction_voxel_matrix_ = std::make_shared<SuctionVoxelMatrix>(suction_rows_count, suction_cols_count,
                                                               active_suction_range_y, active_suction_range_x);
  rosparam_shortcuts::shutdownIfError(parent_name, error);

  return true;
}

void SuctionGraspData::print()
{
  GraspData::print();

  std::cout << "Suction Gripper Parameters: " << std::endl;
  std::cout << "\tactive_suction_range_x_: " << suction_voxel_matrix_->getActiveSuctionWidthY() << std::endl;
  std::cout << "\tactive_suction_range_y_: " << suction_voxel_matrix_->getActiveSuctionWidthX() << std::endl;
}

}  // namespace moveit_grasps
