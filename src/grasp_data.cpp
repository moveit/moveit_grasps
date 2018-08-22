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
#define _USE_MATH_DEFINES

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

// Pose conversion
#include <rviz_visual_tools/rviz_visual_tools.h>

namespace moveit_grasps
{
GraspData::GraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                     moveit::core::RobotModelConstPtr robot_model)
  : base_link_("/base_link"), grasp_depth_(0.12), angle_resolution_(16), robot_model_(robot_model)
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
  std::vector<double> grasp_pose_to_eef_transform;
  double pregrasp_time_from_start;
  double grasp_time_from_start;
  std::string end_effector_name;

  // Helper to let user know what is wrong
  if (!nh.hasParam("base_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader", "Grasp configuration parameter `base_link` missing from rosparam "
                                                "server. Did you load your end effector's configuration yaml file? "
                                                "Searching in namespace: "
                                                    << nh.getNamespace());
    return false;
  }

  // Load all other parameters
  const std::string parent_name = "grasp_data";  // for namespacing logging messages
  rosparam_shortcuts::get(parent_name, nh, "base_link", base_link_);

  // Search within the sub-namespace of this end effector name
  ros::NodeHandle child_nh(nh, end_effector);

  rosparam_shortcuts::get(parent_name, child_nh, "pregrasp_time_from_start", pregrasp_time_from_start);
  rosparam_shortcuts::get(parent_name, child_nh, "grasp_time_from_start", grasp_time_from_start);
  rosparam_shortcuts::get(parent_name, child_nh, "finger_to_palm_depth", finger_to_palm_depth_);
  rosparam_shortcuts::get(parent_name, child_nh, "gripper_finger_width", gripper_finger_width_);
  rosparam_shortcuts::get(parent_name, child_nh, "max_grasp_width", max_grasp_width_);
  rosparam_shortcuts::get(parent_name, child_nh, "grasp_resolution", grasp_resolution_);
  rosparam_shortcuts::get(parent_name, child_nh, "grasp_min_depth", grasp_min_depth_);
  rosparam_shortcuts::get(parent_name, child_nh, "grasp_depth_resolution", grasp_depth_resolution_);
  rosparam_shortcuts::get(parent_name, child_nh, "approach_distance_desired", approach_distance_desired_);
  rosparam_shortcuts::get(parent_name, child_nh, "retreat_distance_desired", retreat_distance_desired_);
  rosparam_shortcuts::get(parent_name, child_nh, "lift_distance_desired", lift_distance_desired_);
  rosparam_shortcuts::get(parent_name, child_nh, "angle_resolution", angle_resolution_);
  rosparam_shortcuts::get(parent_name, child_nh, "end_effector_name", end_effector_name);
  rosparam_shortcuts::get(parent_name, child_nh, "joints", joint_names);
  rosparam_shortcuts::get(parent_name, child_nh, "pregrasp_posture", pre_grasp_posture);
  rosparam_shortcuts::get(parent_name, child_nh, "grasp_posture", grasp_posture);
  rosparam_shortcuts::get(parent_name, child_nh, "grasp_pose_to_eef_transform", grasp_pose_to_eef_transform);
  rosparam_shortcuts::get(parent_name, child_nh, "grasp_padding_on_approach", grasp_padding_on_approach_);
  rosparam_shortcuts::get(parent_name, child_nh, "max_finger_width", max_finger_width_);
  rosparam_shortcuts::get(parent_name, child_nh, "min_finger_width", min_finger_width_);

  // -------------------------------
  // Convert generic grasp pose to this end effector's frame of reference, approach direction for short

  // Orientation
  ROS_ASSERT(grasp_pose_to_eef_transform.size() == 6);
  grasp_pose_to_eef_pose_ = rviz_visual_tools::RvizVisualTools::convertFromXYZRPY(grasp_pose_to_eef_transform, rviz_visual_tools::XYZ);

  // -------------------------------
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
  // -------------------------------
  // Create grasp posture
  grasp_posture_.header.frame_id = base_link_;
  grasp_posture_.header.stamp = ros::Time::now();
  // Name of joints:
  grasp_posture_.joint_names = joint_names;
  // Position of joints
  grasp_posture_.points.resize(1);
  grasp_posture_.points[0].positions = grasp_posture;
  grasp_posture_.points[0].time_from_start = ros::Duration(grasp_time_from_start);

  // -------------------------------
  // Nums
  // distance from center point of object to end effector
  grasp_depth_ = 0.06;  // in negative or 0 this makes the grasps on the other side of the object! (like from below)

  // generate grasps at PI/angle_resolution increments
  // angle_resolution_ = 32; //TODO parametrize this, or move to action interface

  // Copy values from RobotModel
  ee_jmg_ = robot_model_->getJointModelGroup(end_effector_name);
  arm_jmg_ = robot_model_->getJointModelGroup(ee_jmg_->getEndEffectorParentGroup().first);
  parent_link_ = robot_model_->getLinkModel(ee_jmg_->getEndEffectorParentGroup().second);

  ROS_INFO_NAMED("grasp_data", "ee_name: %s, arm_jmg: %s, parent_link: %s",
                                ee_jmg_->getName().c_str(),
                                arm_jmg_->getName().c_str(),
                                parent_link_->getName().c_str());
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
  if (posture.points.size() < 1)
  {
    ROS_ERROR_STREAM_NAMED("grasp_data", "Posture trajectory must have at least 1 waypoint");
    return false;
  }

  // TODO make this more efficient
  // Do for every joint in end effector
  for (std::size_t i = 0; i < posture.joint_names.size(); ++i)
  {
    // Set joint position
    robot_state->setJointPositions(posture.joint_names[i], posture.points[0].positions);
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

  // Data from GDoc: https://docs.google.com/spreadsheets/d/1OXLqzDU7vjZhEis64XW2ziXoY39EwoqGZP6w3LAysvo/edit#gid=0
  static const double SLOPE =
      -6.881728199;  //-0.06881728199; //-14.51428571;  // TODO move this to the yaml file data!!
  static const double INTERCEPT = 0.7097604907;  // 0.7097604907; //10.36703297;
  double joint_position = SLOPE * distance_btw_fingers + INTERCEPT;

  ROS_DEBUG_STREAM_NAMED("grasp_data", "Converted to joint position " << joint_position);

  std::vector<double> joint_positions;
  joint_positions.resize(6);
  joint_positions[0] = joint_position;
  joint_positions[1] = joint_position;

  // JACO SPECIFIC
  static const double FINGER_3_OFFSET = -0.1;  // open more than the others

  // TODO get these values from joint_model, jaco specific
  static const double MIN_JOINT_POSITION = 0.0;
  static const double MAX_JOINT_POSITION = 0.742;

  // special treatment - this joint should be opened more than the others
  joint_positions[2] = joint_position + FINGER_3_OFFSET;
  if (joint_positions[2] < MIN_JOINT_POSITION)
  {
    joint_positions[2] = MIN_JOINT_POSITION;
  }
  if (joint_positions[2] > MAX_JOINT_POSITION)
  {
    joint_positions[2] = MAX_JOINT_POSITION;
  }

  joint_positions[3] = 0;
  joint_positions[4] = 0;
  joint_positions[5] = 0;

  return jointPositionsToGraspPosture(joint_positions, grasp_posture);
}

bool GraspData::jointPositionsToGraspPosture(std::vector<double> joint_positions,
                                             trajectory_msgs::JointTrajectory& grasp_posture)
{
  // ROS_DEBUG_STREAM_NAMED("grasp_data","Moving fingers to joint positions using vector of size "
  //                       << joint_positions.size());

  // TODO (mlautman): This assumes that there is a single joint in the joints array defined in your yaml
  //                  This is a bad assumption. Really this should look at all joints in the end effector
  //                  group individual
  if (grasp_posture_.joint_names.size() < 1)
  {
    ROS_ERROR_STREAM_NAMED("grasp_data", "You must have at least one joint defined in joint_names");
    return false;
  }

  const moveit::core::JointModel* joint = robot_model_->getJointModel(grasp_posture_.joint_names.front());
  const moveit::core::VariableBounds& bound = joint->getVariableBounds()[0];

  for (std::size_t i = 0; i < joint_positions.size(); ++i)
  {
    // Error check
    if (joint_positions[i] > bound.max_position_ || joint_positions[i] < bound.min_position_)
    {
      ROS_ERROR_STREAM_NAMED("grasp_data", "Requested joint " << i << " with value " << joint_positions[i]
                                                              << " is beyond limits of " << bound.min_position_ << ", "
                                                              << bound.max_position_);
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
  std::cout << "grasp_pose_to_eef_pose_: \n" << grasp_pose_to_eef_pose_.translation() << "\n"
            << grasp_pose_to_eef_pose_.rotation() << std::endl;
  std::cout << "pre_grasp_posture_: \n" << pre_grasp_posture_ << std::endl;
  std::cout << "grasp_posture_: \n" << grasp_posture_ << std::endl;
  std::cout << "base_link_: " << base_link_ << std::endl;
  std::cout << "ee_group_: " << ee_jmg_->getName() << std::endl;
  std::cout << "grasp_depth_: " << grasp_depth_ << std::endl;
  std::cout << "angle_resolution_: " << angle_resolution_ << std::endl;
  std::cout << "finger_to_palm_depth_: " << finger_to_palm_depth_ << std::endl;
  std::cout << "grasp_padding_on_approach_: " << grasp_padding_on_approach_ << std::endl;
  std::cout << "max_finger_width_: " << max_finger_width_ << std::endl;
  std::cout << "min_finger_width_: " << min_finger_width_ << std::endl;
}

}  // namespace
