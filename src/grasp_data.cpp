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

/* Authors: Bence Magyar, Dave Coleman
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
#include <rviz_visual_tools/ros_param_utilities.h>

namespace moveit_grasps
{

GraspData::GraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                     moveit::core::RobotModelConstPtr robot_model)
  : base_link_("/base_link")
  , grasp_depth_(0.12)
  , angle_resolution_(16)
{
  if (!loadGraspData(nh, end_effector, robot_model))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data","Error loading grasp data, shutting down");
    exit(-1);
  }
}

bool GraspData::loadGraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                              moveit::core::RobotModelConstPtr robot_model)
{
  std::vector<std::string> joint_names;
  std::vector<double> pre_grasp_posture; // todo: remove all underscore post-fixes
  std::vector<double> grasp_posture;
  std::vector<double> grasp_pose_to_eef_transform;
  double pregrasp_time_from_start;
  double grasp_time_from_start;
  std::string end_effector_name;

  // Helper to let user know what is wrong
  if (!nh.hasParam("base_link"))
  {
    ROS_ERROR_STREAM_NAMED("grasp_data_loader","Grasp configuration parameter `base_link` missing from rosparam server. Did you load your end effector's configuration yaml file? Searching in namespace: " << nh.getNamespace());
    return false;
  }

  // Load all other parameters
  const std::string parent_name = "grasp_data"; // for namespacing logging messages
  rviz_visual_tools::getStringParameter(parent_name, nh, "base_link", base_link_);

  // Search within the sub-namespace of this end effector name
  ros::NodeHandle child_nh(nh, end_effector);

  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "pregrasp_time_from_start", pregrasp_time_from_start);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "grasp_time_from_start", grasp_time_from_start);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "finger_to_palm_depth", finger_to_palm_depth_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "gripper_finger_width", gripper_finger_width_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "max_grasp_width", max_grasp_width_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "grasp_resolution", grasp_resolution_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "grasp_min_depth", grasp_min_depth_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "grasp_depth_resolution", grasp_depth_resolution_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "approach_distance_desired", approach_distance_desired_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "retreat_distance_desired", retreat_distance_desired_);
  rviz_visual_tools::getDoubleParameter(parent_name, child_nh, "lift_distance_desired", lift_distance_desired_);
  rviz_visual_tools::getIntParameter(parent_name, child_nh, "angle_resolution", angle_resolution_);
  rviz_visual_tools::getStringParameter(parent_name, child_nh, "end_effector_name", end_effector_name);
  rviz_visual_tools::getStringParameters(parent_name, child_nh, "joints", joint_names);
  rviz_visual_tools::getDoubleParameters(parent_name, child_nh, "pregrasp_posture", pre_grasp_posture);
  rviz_visual_tools::getDoubleParameters(parent_name, child_nh, "grasp_posture", grasp_posture);
  rviz_visual_tools::getDoubleParameters(parent_name, child_nh, "grasp_pose_to_eef_transform", grasp_pose_to_eef_transform);


  // -------------------------------
  // Convert generic grasp pose to this end effector's frame of reference, approach direction for short

  // Orientation
  ROS_ASSERT(grasp_pose_to_eef_transform.size() == 6);

  Eigen::AngleAxisd rollAngle (grasp_pose_to_eef_transform[3], Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitchAngle(grasp_pose_to_eef_transform[4], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd yawAngle  (grasp_pose_to_eef_transform[5], Eigen::Vector3d::UnitY());
  Eigen::Quaternion<double> quat = rollAngle * yawAngle * pitchAngle;

  grasp_pose_to_eef_pose_ = Eigen::Translation3d(grasp_pose_to_eef_transform[0],
                                                 grasp_pose_to_eef_transform[1],
                                                 grasp_pose_to_eef_transform[2]) * quat;

  // -------------------------------
  // Create pre-grasp posture if specified
  if(!pre_grasp_posture.empty())
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
  grasp_depth_ = 0.06;// in negative or 0 this makes the grasps on the other side of the object! (like from below)

  // generate grasps at PI/angle_resolution increments
  //angle_resolution_ = 32; //TODO parametrize this, or move to action interface

  // Copy values from RobotModel
  ee_jmg_ = robot_model->getJointModelGroup(end_effector_name);
  arm_jmg_ = robot_model->getJointModelGroup(ee_jmg_->getEndEffectorParentGroup().first);
  parent_link_ = robot_model->getLinkModel(ee_jmg_->getEndEffectorParentGroup().second);

  // Debug
  //moveit_grasps::Grasps::printObjectGraspData(grasp_data);

  return true;
}

bool GraspData::setRobotStatePreGrasp( robot_state::RobotStatePtr &robot_state )
{
  return setRobotState( robot_state, pre_grasp_posture_ );
}
bool GraspData::setRobotStateGrasp( robot_state::RobotStatePtr &robot_state )
{
  return setRobotState( robot_state, grasp_posture_ );
}

bool GraspData::setRobotState( robot_state::RobotStatePtr &robot_state, const trajectory_msgs::JointTrajectory &posture )
{
  // Assume joint trajectory has only 1 waypoint
  if (posture.points.size() < 1)
  {
    ROS_ERROR_STREAM_NAMED("grasp_data","Posture trajectory must have at least 1 waypoint");
    return false;
  }

  // Do for every joint in end effector
  for (std::size_t i = 0; i < posture.joint_names.size(); ++i)
  {
    // Set joint position
    robot_state->setJointPositions( posture.joint_names[i],
                                    posture.points[0].positions );
  }
  return true;
}

void GraspData::print()
{
  ROS_WARN_STREAM_NAMED("grasp_data","Debug Grasp Data variable values:");
  std::cout << "grasp_pose_to_eef_pose_: \n" << grasp_pose_to_eef_pose_.translation() << "\n" << grasp_pose_to_eef_pose_.rotation() <<std::endl;
  std::cout << "pre_grasp_posture_: \n" << pre_grasp_posture_ << std::endl;
  std::cout << "grasp_posture_: \n" << grasp_posture_ << std::endl;
  std::cout << "base_link_: " << base_link_ << std::endl;
  std::cout << "ee_group_: " << ee_jmg_->getName()  <<  std::endl;
  std::cout << "grasp_depth_: " << grasp_depth_ << std::endl;
  std::cout << "angle_resolution_: " << angle_resolution_ << std::endl;
  std::cout << "finger_to_palm_depth_: " << finger_to_palm_depth_ << std::endl;
}

} // namespace
