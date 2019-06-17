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

#include <moveit_grasps/grasp_data.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// C++
#include <math.h>
// TODO(davetcoleman): remove this
#define _USE_MATH_DEFINES

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Pose conversion
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace moveit_grasps
{
GraspData::GraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                     moveit::core::RobotModelConstPtr robot_model)
  : base_link_("/base_link"), robot_model_(robot_model)
{
  if (!loadGraspData(nh, end_effector))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data", "Error loading grasp data, shutting down");
    exit(-1);
  }
}

bool GraspData::loadGraspData(const ros::NodeHandle& nh, const std::string& end_effector)
{
  std::vector<std::string> joint_names;
  std::vector<double> pre_grasp_posture;  // todo: remove all underscore post-fixes
  std::vector<double> grasp_posture;
  double pregrasp_time_from_start;
  double grasp_time_from_start;
  std::string end_effector_name;

  // Helper to let user know what is wrong
  if (!nh.hasParam("base_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data", "Grasp configuration parameter `base_link` missing from rosparam "
                                         "server. Did you load your end effector's configuration yaml file? "
                                         "Searching in namespace: "
                                             << nh.getNamespace());
    return false;
  }

  // Load all other parameters
  const std::string parent_name = "grasp_data";  // for namespacing logging messages
  std::size_t error = 0;
  error += !rosparam_shortcuts::get(parent_name, nh, "base_link", base_link_);

  // Search within the sub-namespace of this end effector name
  ros::NodeHandle child_nh(nh, end_effector);

  error += !rosparam_shortcuts::get(parent_name, child_nh, "pregrasp_time_from_start", pregrasp_time_from_start);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "grasp_time_from_start", grasp_time_from_start);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "grasp_resolution", grasp_resolution_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "grasp_min_depth", grasp_min_depth_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "grasp_max_depth", grasp_max_depth_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "grasp_depth_resolution", grasp_depth_resolution_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "approach_distance_desired", approach_distance_desired_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "retreat_distance_desired", retreat_distance_desired_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "lift_distance_desired", lift_distance_desired_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "angle_resolution", angle_resolution_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "end_effector_name", end_effector_name);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "joints", joint_names);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "pregrasp_posture", pre_grasp_posture);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "grasp_posture", grasp_posture);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "tcp_to_eef_mount_transform", tcp_to_eef_mount_);
  error += !rosparam_shortcuts::get(parent_name, child_nh, "grasp_padding_on_approach", grasp_padding_on_approach_);

  // Find out if the end effector uses suction or fingers (NOTE: must be one of 'finger' or 'suction')
  std::string end_effector_type_str;
  child_nh.param<std::string>("end_effector_type", end_effector_type_str, "finger");

  if (end_effector_type_str == "finger")
  {
    end_effector_type_ = FINGER;
  }
  else if (end_effector_type_str == "suction")
  {
    end_effector_type_ = SUCTION;
  }
  else
  {
    ROS_ASSERT_MSG(false, "Unrecognized end effector type: %s", end_effector_type_str.c_str());
  }

  if (end_effector_type_ == FINGER)
  {
    error += !rosparam_shortcuts::get(parent_name, child_nh, "gripper_finger_width", gripper_finger_width_);
    error += !rosparam_shortcuts::get(parent_name, child_nh, "max_grasp_width", max_grasp_width_);
    error += !rosparam_shortcuts::get(parent_name, child_nh, "max_finger_width", max_finger_width_);
    error += !rosparam_shortcuts::get(parent_name, child_nh, "min_finger_width", min_finger_width_);
  }
  else if (end_effector_type_ == SUCTION)
  {
    int suction_rows_count, suction_cols_count;
    error += !rosparam_shortcuts::get(parent_name, child_nh, "active_suction_range_x", active_suction_range_x_);
    error += !rosparam_shortcuts::get(parent_name, child_nh, "active_suction_range_y", active_suction_range_y_);
    child_nh.param<int>("suction_rows_count", suction_rows_count, 1);
    child_nh.param<int>("suction_cols_count", suction_cols_count, 1);
    suction_voxel_matrix_.reset(new SuctionVoxelMatrix(suction_rows_count, suction_cols_count, active_suction_range_y_,
                                                       active_suction_range_x_));
  }
  rosparam_shortcuts::shutdownIfError(parent_name, error);

  // Convert generic grasp pose to this end effector's frame of reference, approach direction for short

  // Create pre-grasp posture if specified
  if (!pre_grasp_posture.empty())
  {
    pre_grasp_posture_.header.frame_id = base_link_;
    pre_grasp_posture_.header.stamp = ros::Time::now();
    // Name of joints:
    pre_grasp_posture_.joint_names = joint_names;
    // Position of joints
    pre_grasp_posture_.points.resize(1);
    pre_grasp_posture_.points[0].positions = pre_grasp_posture;
    pre_grasp_posture_.points[0].time_from_start = ros::Duration(pregrasp_time_from_start);
  }

  // Create grasp posture
  grasp_posture_.header.frame_id = base_link_;
  grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_posture_.joint_names = joint_names;
  // Position of joints
  grasp_posture_.points.resize(1);
  grasp_posture_.points[0].positions = grasp_posture;
  grasp_posture_.points[0].time_from_start = ros::Duration(grasp_time_from_start);

  // Copy values from RobotModel
  ee_jmg_ = robot_model_->getJointModelGroup(end_effector_name);
  arm_jmg_ = robot_model_->getJointModelGroup(ee_jmg_->getEndEffectorParentGroup().first);
  parent_link_ = robot_model_->getLinkModel(ee_jmg_->getEndEffectorParentGroup().second);

  ROS_INFO_NAMED("grasp_data", "ee_name: %s, arm_jmg: %s, parent_link: %s", ee_jmg_->getName().c_str(),
                 arm_jmg_->getName().c_str(), parent_link_->getName().c_str());
  return true;
}

bool GraspData::setRobotStatePreGrasp(robot_state::RobotStatePtr& robot_state)
{
  ROS_WARN_STREAM_NAMED("grasp_data", "setRobotStatePreGrasp is probably wrong");
  return setRobotState(robot_state, pre_grasp_posture_);
}

bool GraspData::setRobotStateGrasp(robot_state::RobotStatePtr& robot_state)
{
  ROS_WARN_STREAM_NAMED("grasp_data", "setRobotStateGrasp is probably wrong");
  return setRobotState(robot_state, grasp_posture_);
}

bool GraspData::setRobotState(robot_state::RobotStatePtr& robot_state, const trajectory_msgs::JointTrajectory& posture)
{
  // Assume joint trajectory has only 1 waypoint
  if (end_effector_type_ == FINGER)
  {
    if (posture.points.size() < 1)
    {
      ROS_ERROR_STREAM_NAMED("grasp_data", "Posture trajectory for finger'd grasper must have at least 1 waypoint");
      return false;
    }

    // TODO(davetcoleman): make this more efficient
    // Do for every joint in end effector
    for (std::size_t i = 0; i < posture.joint_names.size(); ++i)
    {
      // Set joint position
      robot_state->setJointPositions(posture.joint_names[i], posture.points[0].positions);
    }
  }
  return true;
}

bool GraspData::setGraspWidth(const double& percent_open, const double& min_finger_width,
                              trajectory_msgs::JointTrajectory& grasp_posture)
{
  if (percent_open < 0 || percent_open > 1)
  {
    ROS_ERROR_STREAM_NAMED("grasp_data", "Invalid percentage passed in " << percent_open);
    return false;
  }

  // Ensure min_finger_width is not less than actual min finger width
  double min_finger_width_adjusted = std::max(min_finger_width, min_finger_width_);

  // Max width = max_finger_width_
  // Min width = min_finger_width_adjusted
  double distance_btw_fingers =
      min_finger_width_adjusted + (max_finger_width_ - min_finger_width_adjusted) * percent_open;
  return fingerWidthToGraspPosture(distance_btw_fingers, grasp_posture);
}

bool GraspData::fingerWidthToGraspPosture(const double& distance_btw_fingers,
                                          trajectory_msgs::JointTrajectory& grasp_posture)
{
  // TODO(mlautman): Change this function to take in a method for translating joint values to grasp width
  //       Currently this function simply interpolates between max open and max closed
  ROS_DEBUG_STREAM_NAMED("grasp_data", "Setting grasp posture to have distance_between_fingers of "
                                           << distance_btw_fingers);

  // Error check
  const double EPSILON = 0.0000001;
  if (distance_btw_fingers > max_finger_width_ + EPSILON || distance_btw_fingers < min_finger_width_ - EPSILON)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_data", "Requested " << distance_btw_fingers << " is beyond limits of "
                                                      << min_finger_width_ << "," << max_finger_width_);
    return false;
  }

  // NOTE: We assume a linear relationship between the actuated joint values and the distance between fingers.
  //       This is probably incorrect but until we expose an interface for passing in a function to translate from
  //       joint values to grasp width, it's the best we got...
  // TODO(mlautman): Make it so that a user can pass in a function here.
  std::vector<std::string> joint_names = pre_grasp_posture_.joint_names;
  std::vector<double> grasp_pose = grasp_posture_.points[0].positions;
  std::vector<double> pre_grasp_pose = pre_grasp_posture_.points[0].positions;
  if (joint_names.size() != grasp_pose.size() || grasp_pose.size() != pre_grasp_pose.size())
  {
    ROS_ERROR_NAMED("grasp_data", "Mismatched vector sizes joint_names.size()=%zu, grasp_pose.size()=%zu, and "
                                  "pre_grasp_pose.size()=%zu",
                    joint_names.size(), grasp_pose.size(), pre_grasp_pose.size());
    return false;
  }

  std::vector<double> slope(joint_names.size());
  std::vector<double> intercept(joint_names.size());
  std::vector<double> joint_positions(joint_names.size());
  for (std::size_t joint_index = 0; joint_index < joint_names.size(); joint_index++)
  {
    slope[joint_index] =
        (max_finger_width_ - min_finger_width_) / (pre_grasp_pose[joint_index] - grasp_pose[joint_index]);
    intercept[joint_index] = max_finger_width_ - slope[joint_index] * pre_grasp_pose[joint_index];

    // Sanity check
    ROS_ASSERT_MSG(intercept[joint_index] == min_finger_width_ - slope[joint_index] * grasp_pose[joint_index],
                   "we got different y intercept!! %.3f and %.3f", intercept[joint_index],
                   min_finger_width_ - slope[joint_index] * grasp_pose[joint_index]);

    joint_positions[joint_index] = (distance_btw_fingers - intercept[joint_index]) / slope[joint_index];

    ROS_DEBUG_NAMED("grasp_data", "Converted joint %s to position %.3f", joint_names[joint_index].c_str(),
                    joint_positions[joint_index]);
  }

  return jointPositionsToGraspPosture(joint_positions, grasp_posture);
}

bool GraspData::jointPositionsToGraspPosture(std::vector<double> joint_positions,
                                             trajectory_msgs::JointTrajectory& grasp_posture)
{
  for (std::size_t joint_index = 0; joint_index < pre_grasp_posture_.joint_names.size(); joint_index++)
  {
    const moveit::core::JointModel* joint = robot_model_->getJointModel(pre_grasp_posture_.joint_names[joint_index]);
    ROS_ASSERT_MSG(joint->getVariableBounds().size() > 0, "joint->getVariableBounds() is empty for %s",
                   pre_grasp_posture_.joint_names[joint_index].c_str());
    const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

    if (joint_positions[joint_index] > bound.max_position_ || joint_positions[joint_index] < bound.min_position_)
    {
      ROS_ERROR_STREAM_NAMED("grasp_data", "Requested joint " << pre_grasp_posture_.joint_names[joint_index].c_str()
                                                              << "at index" << joint_index << " with value "
                                                              << joint_positions[joint_index] << " is beyond limits of "
                                                              << bound.min_position_ << ", " << bound.max_position_);
      return false;
    }
  }

  // Get default grasp posture
  grasp_posture = grasp_posture_;

  // Error check
  if (joint_positions.size() != grasp_posture.points.front().positions.size())
  {
    ROS_ERROR_STREAM_NAMED("grasp_data",
                           "Not enough finger joints passed in: " << joint_positions.size() << " positions but expect "
                                                                  << grasp_posture.points.front().positions.size());
    return false;
  }

  // Set joint positions
  grasp_posture.points.front().positions = joint_positions;

  return true;
}

void GraspData::print()
{
  ROS_WARN_STREAM_NAMED("grasp_data", "Debug Grasp Data variable values:");
  std::cout << "tcp_to_eef_mount_: \n"
            << tcp_to_eef_mount_.translation() << "\n"
            << tcp_to_eef_mount_.rotation() << std::endl;
  std::cout << "pre_grasp_posture_: \n" << pre_grasp_posture_ << std::endl;
  std::cout << "grasp_posture_: \n" << grasp_posture_ << std::endl;
  std::cout << "base_link_: " << base_link_ << std::endl;
  std::cout << "ee_group_: " << ee_jmg_->getName() << std::endl;
  std::cout << "angle_resolution_: " << angle_resolution_ << std::endl;
  std::cout << "grasp_max_depth_: " << grasp_max_depth_ << std::endl;
  std::cout << "grasp_padding_on_approach_: " << grasp_padding_on_approach_ << std::endl;

  if (end_effector_type_ == FINGER)
  {
    std::cout << "Finger Gripper Parameters: " << std::endl;
    std::cout << "\tgripper_finger_width_: " << gripper_finger_width_ << std::endl;
    std::cout << "\tmax_grasp_width_: " << max_grasp_width_ << std::endl;
    std::cout << "\tmax_finger_width_: " << max_finger_width_ << std::endl;
    std::cout << "\tmin_finger_width_: " << min_finger_width_ << std::endl;
  }
  else if (end_effector_type_ == SUCTION)
  {
    std::cout << "Suction Gripper Parameters: " << std::endl;
    std::cout << "\tactive_suction_range_x_: " << active_suction_range_x_ << std::endl;
    std::cout << "\tactive_suction_range_y_: " << active_suction_range_y_ << std::endl;
  }
  else
  {
    std::cout << "end_effector_type_ is invalid!! " << end_effector_type_ << std::endl;
  }
}

}  // namespace
