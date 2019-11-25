/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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

/* Author: Dave Coleman <dave@picknik.ai>, Andy McEvoy
   Desc:   Generates geometric grasps for cuboids and blocks, not using physics or contact wrenches
*/

#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_filter.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace
{
void debugFailedOpenGripper(double percent_open, double min_finger_open_on_approach, double object_width,
                            double grasp_padding_on_approach)
{
  ROS_ERROR_STREAM_NAMED("grasp_generator", "Unable to set grasp width to "
                                                << percent_open << " % open. Stats:"
                                                << "\n min_finger_open_on_approach: \t " << min_finger_open_on_approach
                                                << "\n object_width: \t " << object_width
                                                << "\n grasp_padding_on_approach_: \t " << grasp_padding_on_approach);
}

}  // namespace

namespace moveit_grasps

{
// Constructor
GraspGenerator::GraspGenerator(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, bool verbose)
  : ideal_grasp_pose_(Eigen::Isometry3d::Identity())
  , visual_tools_(visual_tools)
  , verbose_(verbose)
  , nh_("~/moveit_grasps/generator")
  , grasp_score_weights_(std::make_shared<GraspScoreWeights>())
{
  // Load visulization settings
  const std::string parent_name = "grasps";  // for namespacing logging messages
  std::size_t error = 0;

  error += !rosparam_shortcuts::get(parent_name, nh_, "verbose", verbose_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_prefiltered_grasps", show_prefiltered_grasps_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_prefiltered_grasps_speed", show_prefiltered_grasps_speed_);

  // Load scoring weights
  rosparam_shortcuts::shutdownIfError(parent_name, error);
}

void GraspGenerator::setIdealTCPGraspPoseRPY(const std::vector<double>& ideal_grasp_orientation_rpy)
{
  ROS_ASSERT_MSG(ideal_grasp_orientation_rpy.size() == 3, "setIdealTCPGraspPoseRPY must be set with a vector of length "
                                                          "3");

  // copy the ideal_grasp_pose.translation() so that we only change the orientation.
  Eigen::Vector3d ideal_grasp_pose_translation(ideal_grasp_pose_.translation());

  // Set ideal grasp pose (currently only uses orientation of pose)
  ideal_grasp_pose_ = Eigen::Isometry3d::Identity() *
                      Eigen::AngleAxisd(ideal_grasp_orientation_rpy[0], Eigen::Vector3d::UnitX()) *
                      Eigen::AngleAxisd(ideal_grasp_orientation_rpy[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(ideal_grasp_orientation_rpy[2], Eigen::Vector3d::UnitZ());

  ideal_grasp_pose_.translation() = ideal_grasp_pose_translation;
}

Eigen::Vector3d GraspGenerator::getPreGraspDirection(const moveit_msgs::Grasp& grasp, const std::string& ee_parent_link)
{
  // Grasp Pose Variables
  Eigen::Isometry3d grasp_pose_eef_mount_eigen;
  tf::poseMsgToEigen(grasp.grasp_pose.pose, grasp_pose_eef_mount_eigen);

  // The direction of the pre-grasp in the frame of the parent link
  Eigen::Vector3d pre_grasp_approach_direction =
      Eigen::Vector3d(grasp.pre_grasp_approach.direction.vector.x, grasp.pre_grasp_approach.direction.vector.y,
                      grasp.pre_grasp_approach.direction.vector.z);

  // Decide if we need to change the approach_direction to the local frame of the end effector orientation
  if (grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link)
  {
    // Apply/compute the approach_direction vector in the local frame of the grasp_pose orientation
    return grasp_pose_eef_mount_eigen.rotation() * pre_grasp_approach_direction;
  }
  return pre_grasp_approach_direction;
}

geometry_msgs::PoseStamped GraspGenerator::getPreGraspPose(const GraspCandidatePtr& grasp_candidate,
                                                           const std::string& ee_parent_link)
{
  // Grasp Pose Variables
  Eigen::Isometry3d grasp_pose_eef_mount_eigen;
  tf::poseMsgToEigen(grasp_candidate->grasp_.grasp_pose.pose, grasp_pose_eef_mount_eigen);

  // Get pre-grasp pose first
  Eigen::Isometry3d pre_grasp_pose_eef_mount_eigen =
      grasp_pose_eef_mount_eigen;  // Copy original grasp pose to pre-grasp pose

  // Approach direction
  Eigen::Vector3d pre_grasp_approach_direction_local = getPreGraspDirection(grasp_candidate->grasp_, ee_parent_link);

  // Update the grasp matrix usign the new locally-framed approach_direction
  pre_grasp_pose_eef_mount_eigen.translation() -=
      pre_grasp_approach_direction_local * grasp_candidate->grasp_.pre_grasp_approach.desired_distance;

  // Convert eigen pre-grasp position back to regular message
  geometry_msgs::PoseStamped pre_grasp_pose_eef_mount_msg;
  tf::poseEigenToMsg(pre_grasp_pose_eef_mount_eigen, pre_grasp_pose_eef_mount_msg.pose);

  // Copy original header to new grasp
  pre_grasp_pose_eef_mount_msg.header = grasp_candidate->grasp_.grasp_pose.header;

  return pre_grasp_pose_eef_mount_msg;
}

void GraspGenerator::getGraspWaypoints(const GraspCandidatePtr& grasp_candidate,
                                       EigenSTL::vector_Isometry3d& grasp_waypoints)
{
  Eigen::Isometry3d grasp_pose;
  tf::poseMsgToEigen(grasp_candidate->grasp_.grasp_pose.pose, grasp_pose);

  const geometry_msgs::PoseStamped pregrasp_pose_msg =
      GraspGenerator::getPreGraspPose(grasp_candidate, grasp_candidate->grasp_data_->parent_link_->getName());

  // Create waypoints
  Eigen::Isometry3d pregrasp_pose;
  tf::poseMsgToEigen(pregrasp_pose_msg.pose, pregrasp_pose);

  Eigen::Isometry3d lifted_grasp_pose = grasp_pose;
  lifted_grasp_pose.translation().z() += grasp_candidate->grasp_data_->lift_distance_desired_;

  // Solve for post grasp retreat
  Eigen::Isometry3d retreat_pose = lifted_grasp_pose;
  Eigen::Vector3d postgrasp_vector(grasp_candidate->grasp_.post_grasp_retreat.direction.vector.x,
                                   grasp_candidate->grasp_.post_grasp_retreat.direction.vector.y,
                                   grasp_candidate->grasp_.post_grasp_retreat.direction.vector.z);
  postgrasp_vector.normalize();

  retreat_pose.translation() +=
      retreat_pose.rotation() * postgrasp_vector * grasp_candidate->grasp_.post_grasp_retreat.desired_distance;

  grasp_waypoints.clear();
  grasp_waypoints.resize(4);
  grasp_waypoints[0] = pregrasp_pose;
  grasp_waypoints[1] = grasp_pose;
  grasp_waypoints[2] = lifted_grasp_pose;
  grasp_waypoints[3] = retreat_pose;
}

void GraspGenerator::publishGraspArrow(const geometry_msgs::Pose& grasp, const GraspDataPtr& grasp_data,
                                       const rviz_visual_tools::colors& color, double approach_length)
{
  visual_tools_->publishArrow(grasp, color, rviz_visual_tools::MEDIUM);
}

bool GraspGenerator::visualizeAnimatedGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates,
                                             const moveit::core::JointModelGroup* ee_jmg, double animation_speed)
{
  // Convert the grasp_candidates into a format moveit_visual_tools can use
  std::vector<moveit_msgs::Grasp> grasps;
  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    grasps.push_back(grasp_candidates[i]->grasp_);
  }

  return visual_tools_->publishAnimatedGrasps(grasps, ee_jmg, show_prefiltered_grasps_speed_);
}

}  // namespace moveit_grasps
