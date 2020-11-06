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

/* Author: Dave Coleman <dave@picknik.ai>
   Desc:   Filters grasps based on kinematic feasibility and collision
*/

// moveit_grasps
#include <moveit_grasps/suction_grasp_filter.h>
#include <moveit_grasps/suction_grasp_candidate.h>
#include <moveit_grasps/state_validity_callback.h>

// moveit
#include <moveit/robot_state/conversions.h>
#include <moveit/transforms/transforms.h>
#include <moveit/collision_detection/collision_tools.h>
#include <memory>

// Eigen
#include <eigen_conversions/eigen_msg.h>
#include <tf2_eigen/tf2_eigen.h>

namespace moveit_grasps
{
// Constructor
SuctionGraspFilter::SuctionGraspFilter(const robot_state::RobotStatePtr& robot_state,
                                       const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools)
  : GraspFilter::GraspFilter(robot_state, visual_tools), name_("suction_grasp_filter"), suction_voxel_overlap_cutoff_(0)
{
}

bool SuctionGraspFilter::filterGraspsBySuctionVoxelOverlap(std::vector<GraspCandidatePtr>& grasp_candidates)
{
  static const std::string logger_name = name_ + ".filter_grasps_by_suction_voxel_overlap";

  // Pre-filter for suction voxel overlap
  std::size_t count = 0;
  std::size_t valid_grasps = 0;
  for (std::size_t ix = 0; ix < grasp_candidates.size(); ++ix)
  {
    ROS_DEBUG_STREAM_NAMED(logger_name, "---------------\nGrasp candidate: " << count++);
    bool valid = false;
    auto suction_grasp_candidate = std::dynamic_pointer_cast<SuctionGraspCandidate>(grasp_candidates[ix]);
    if (!suction_grasp_candidate)
    {
      ROS_ERROR_NAMED(logger_name, "grasp_candidate is not castable as SuctionGraspCandidatePtr");
      return 0;
    }

    std::vector<bool> suction_voxel_enabled =
        suction_grasp_candidate->getSuctionVoxelEnabled(suction_voxel_overlap_cutoff_);
    for (const bool& voxel_enabled : suction_voxel_enabled)
    {
      if (voxel_enabled)
      {
        valid = true;
        ++valid_grasps;
        break;
      }
    }
    if (!valid)
    {
      grasp_candidates[ix]->grasp_filtered_code_ = SuctionGraspFilterCode::GRASP_FILTERED_BY_SUCTION_VOXEL_OVERLAP;
    }
  }
  return valid_grasps;
}

std::size_t SuctionGraspFilter::filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                                   const planning_scene::PlanningScenePtr& planning_scene,
                                                   const robot_model::JointModelGroup* arm_jmg,
                                                   const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp,
                                                   bool visualize, const std::string& target_object_id)
{
  filterGraspsBySuctionVoxelOverlap(grasp_candidates);
  return GraspFilter::filterGraspsHelper(grasp_candidates, planning_scene, arm_jmg, seed_state, filter_pregrasp,
                                         visualize, target_object_id);
}

void SuctionGraspFilter::printFilterStatistics(const std::vector<GraspCandidatePtr>& grasp_candidates) const
{
  static const std::string logger_name = GraspFilter::name_ + ".filter_statistics";

  if (!statistics_verbose_)
    return;

  GraspFilter::printFilterStatistics(grasp_candidates);

  // Count number of grasps remaining
  std::size_t grasp_filtered_by_suction_voxel_overlap = 0;

  for (std::size_t i = 0; i < grasp_candidates.size(); ++i)
  {
    if (grasp_candidates[i]->grasp_filtered_code_ == SuctionGraspFilterCode::GRASP_FILTERED_BY_SUCTION_VOXEL_OVERLAP)
      ++grasp_filtered_by_suction_voxel_overlap;
  }

  ROS_INFO_STREAM_NAMED(logger_name, "-------------------------------------------------------");
  ROS_INFO_STREAM_NAMED(logger_name, "grasp_filtered_by_suction_voxel_overlap            "
                                         << grasp_filtered_by_suction_voxel_overlap);
  ROS_INFO_STREAM_NAMED(logger_name, "-------------------------------------------------------");
}

bool SuctionGraspFilter::processCandidateGrasp(const IkThreadStructPtr& ik_thread_struct)
{
  static const std::string logger_name = name_ + ".process_candidate_grasp";

  // Helper pointer
  GraspCandidatePtr& grasp_candidate = ik_thread_struct->grasp_candidates_[ik_thread_struct->grasp_id];

  auto suction_grasp_data = std::dynamic_pointer_cast<SuctionGraspData>(grasp_candidate->grasp_data_);
  if (!suction_grasp_data)
  {
    ROS_ERROR_STREAM_NAMED(logger_name, "Could not cast GraspCandidatePtr->GraspDataPtr as "
                                        "SuctionGraspDataPtr");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_INVALID;
    return false;
  }

  auto suction_grasp_candidate = std::dynamic_pointer_cast<SuctionGraspCandidate>(grasp_candidate);
  if (!suction_grasp_candidate)
  {
    ROS_ERROR_STREAM_NAMED(logger_name, "Could not cast GraspCandidatePtr as "
                                        "SuctionGraspCandidatePtr");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_INVALID;
    return false;
  }

  std::vector<std::string> collision_object_names;
  if (!attachActiveSuctionCupCO(suction_grasp_data,
                                suction_grasp_candidate->getSuctionVoxelEnabled(suction_voxel_overlap_cutoff_),
                                ik_thread_struct->planning_scene_, collision_object_names))
  {
    ROS_ERROR_STREAM_NAMED(logger_name, "Failed to attch active suction cups as collision objects in the planning "
                                        "scene");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_INVALID;
    return false;
  }

  if (!ik_thread_struct->grasp_target_object_id_.empty() && !collision_object_names.empty())
  {
    setACMFingerEntry(ik_thread_struct->grasp_target_object_id_, true, collision_object_names,
                      ik_thread_struct->planning_scene_);
  }

  bool filter_results = GraspFilter::processCandidateGrasp(ik_thread_struct);

  // Cleanup ACM changes
  if (!ik_thread_struct->grasp_target_object_id_.empty() && !collision_object_names.empty())
  {
    setACMFingerEntry(ik_thread_struct->grasp_target_object_id_, false, collision_object_names,
                      ik_thread_struct->planning_scene_);
  }

  if (!removeAllSuctionCupCO(suction_grasp_data, ik_thread_struct->planning_scene_))
  {
    ROS_ERROR_STREAM_NAMED(logger_name, "Failed to detach all active suction cups");
    grasp_candidate->grasp_filtered_code_ = GraspFilterCode::GRASP_INVALID;
    return false;
  }

  if (!filter_results)
  {
    ROS_DEBUG_STREAM_NAMED(logger_name, "Candidate grasp invalid");
    return false;
  }

  return true;
}

std::string SuctionGraspFilter::suctionVoxelIxToCollisionObjectId(std::size_t voxel_ix)
{
  return "suction_voxel_" + std::to_string(voxel_ix);
}

bool SuctionGraspFilter::removeAllSuctionCupCO(const SuctionGraspDataPtr& grasp_data,
                                               const planning_scene::PlanningScenePtr& planning_scene)
{
  static const std::string logger_name = name_ + ".remove_all_suction_cup_co";

  // Get the attach link
  std::string ik_link;
  if (!grasp_data->tcp_name_.empty())
  {
    ik_link = grasp_data->tcp_name_;
  }
  else
  {
    ROS_WARN_STREAM_NAMED(logger_name, "It is strongly encouraged to define the tcp link by name");
    ik_link = grasp_data->parent_link_->getName();
  }

  // Create an AttachedCollisionObject
  moveit_msgs::AttachedCollisionObject suction_voxel_aco;

  // Set the aco attached link name
  suction_voxel_aco.link_name = ik_link;

  // Create a reference to the collision object for convenience
  moveit_msgs::CollisionObject& suction_voxel_co = suction_voxel_aco.object;

  // Mark object to be removed
  suction_voxel_co.operation = moveit_msgs::CollisionObject::REMOVE;

  // Get EE_JMG link names for setting ACM enabled / disabled
  std::vector<std::string> ee_links = grasp_data->ee_jmg_->getLinkModelNames();

  // Iterate through the suction voxels and remove any that exist in the PS.
  std::size_t num_voxels = grasp_data->suction_voxel_matrix_->getNumVoxels();
  for (std::size_t voxel_ix = 0; voxel_ix < num_voxels; ++voxel_ix)
  {
    // Set the aco name
    suction_voxel_co.id = suctionVoxelIxToCollisionObjectId(voxel_ix);
    setACMFingerEntry(suction_voxel_co.id, false, ee_links, planning_scene);

    // Check if the ACO already exists
    moveit_msgs::AttachedCollisionObject aco;
    if (planning_scene->getAttachedCollisionObjectMsg(aco, suction_voxel_aco.object.id))
    {
      ROS_DEBUG_STREAM_NAMED(logger_name, "Removing ACO: " << suction_voxel_aco.object.id);
      // Dettach the collision object
      if (!planning_scene->processAttachedCollisionObjectMsg(suction_voxel_aco))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Failed to processACOMsg for: " << suction_voxel_aco.object.id);
        return false;
      }

      // Check if the collision object was successfully detached
      aco = moveit_msgs::AttachedCollisionObject();
      if (planning_scene->getAttachedCollisionObjectMsg(aco, suction_voxel_aco.object.id))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Failed to detach object: " << suction_voxel_aco.object.id);
        ROS_DEBUG_STREAM_NAMED(logger_name, "Found : \n" << aco);
        return false;
      }
    }
    // Check if the collision object exists and remove it
    moveit_msgs::CollisionObject co;
    if (planning_scene->getCollisionObjectMsg(co, suction_voxel_co.id))
    {
      ROS_DEBUG_STREAM_NAMED(logger_name, "Removing CO: " << suction_voxel_co.id);
      // Remove Collision object from planning scene
      if (!planning_scene->processCollisionObjectMsg(suction_voxel_co))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Failed to processCollisionObjectMsg for: " << suction_voxel_co.id);
        return false;
      }

      // Check to make sure the object is no longer in the planning scene
      co = moveit_msgs::CollisionObject();
      if (planning_scene->getCollisionObjectMsg(co, suction_voxel_co.id))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Failed to remove object " << suction_voxel_co.id << " from planning scene");
        ROS_DEBUG_STREAM_NAMED(logger_name, "Found : \n" << co);
        return false;
      }
    }
  }
  return true;
}

bool SuctionGraspFilter::attachActiveSuctionCupCO(const SuctionGraspDataPtr& grasp_data,
                                                  const std::vector<bool>& suction_voxel_enabled,
                                                  const planning_scene::PlanningScenePtr& planning_scene,
                                                  std::vector<std::string>& collision_object_names)
{
  static const std::string logger_name = name_ + ".attach_active_suction_cup_co";

  // Handle both cases where the grasp_data defines TCP by transform or by frame_id
  Eigen::Isometry3d ik_link_to_tcp = Eigen::Isometry3d::Identity();
  std::string ik_link;
  if (!grasp_data->tcp_name_.empty())
  {
    ik_link = grasp_data->tcp_name_;
  }
  else
  {
    ROS_WARN_STREAM_NAMED(logger_name, "It is strongly encouraged to define the tcp link by name");
    ik_link = grasp_data->parent_link_->getName();
    ik_link_to_tcp = grasp_data->tcp_to_eef_mount_;
  }

  std::size_t num_voxels = grasp_data->suction_voxel_matrix_->getNumVoxels();
  collision_object_names.resize(num_voxels);

  ROS_DEBUG_STREAM_NAMED(logger_name, "~~~~~~~~~~~");
  for (std::size_t ix = 0; ix < suction_voxel_enabled.size(); ++ix)
    ROS_DEBUG_STREAM_NAMED(logger_name, "voxel_" << ix << ":\t" << suction_voxel_enabled[ix]);

  // Get EE_JMG link names for setting ACM enabled / disabled
  std::vector<std::string> ee_links = grasp_data->ee_jmg_->getLinkModelNames();

  for (std::size_t voxel_ix = 0; voxel_ix < num_voxels; ++voxel_ix)
  {
    std::shared_ptr<SuctionVoxel> suction_voxel;
    if (!grasp_data->suction_voxel_matrix_->getSuctionVoxel(voxel_ix, suction_voxel))
    {
      ROS_ERROR_STREAM_NAMED(logger_name, "Invalid suction voxel id: " << voxel_ix);
      return false;
    }
    // Assign collision object names for output
    collision_object_names[voxel_ix] = suctionVoxelIxToCollisionObjectId(voxel_ix);
    setACMFingerEntry(collision_object_names[voxel_ix], true, ee_links, planning_scene);

    // Create an AttachedCollisionObject
    moveit_msgs::AttachedCollisionObject suction_voxel_aco;

    // Create a reference to the collision object for convenience
    moveit_msgs::CollisionObject& suction_voxel_co = suction_voxel_aco.object;

    suction_voxel_co.id = collision_object_names[voxel_ix];

    suction_voxel_co.header.frame_id = ik_link;

    suction_voxel_co.primitives.resize(1);
    suction_voxel_co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    suction_voxel_co.primitives[0].dimensions.resize(3);
    suction_voxel_co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = suction_voxel->x_width_;
    suction_voxel_co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = suction_voxel->y_width_;
    suction_voxel_co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = grasp_data->grasp_max_depth_;

    // The set the attached object pose
    suction_voxel_co.primitive_poses.resize(1);
    Eigen::Isometry3d ik_link_to_voxel_center = ik_link_to_tcp * Eigen::Translation3d(suction_voxel->center_point_) *
                                                Eigen::Translation3d(0, 0, -grasp_data->grasp_max_depth_ / 2.0 + .01);
    suction_voxel_co.primitive_poses[0] = tf2::toMsg(ik_link_to_voxel_center);

    // Set the aco attached link name
    suction_voxel_aco.link_name = ik_link;

    // Check if the ACO already exists
    moveit_msgs::AttachedCollisionObject aco;
    bool aco_exists = planning_scene->getAttachedCollisionObjectMsg(aco, suction_voxel_aco.object.id);

    // If the suction voxel is not attached to the robot in the planning scene but should be
    if (suction_voxel_enabled[voxel_ix] && !aco_exists)
    {
      // Mark object to be added
      suction_voxel_co.operation = moveit_msgs::CollisionObject::ADD;

      // Attach the collision object
      if (!planning_scene->processAttachedCollisionObjectMsg(suction_voxel_aco))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Failed to process processAttachedCollisionObjectMsg for: "
                                               << suction_voxel_aco.object.id);
        return false;
      }

      // Check if the collision object was successfully attached
      if (!planning_scene->getAttachedCollisionObjectMsg(aco, suction_voxel_aco.object.id))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Object: " << suction_voxel_aco.object.id << " not attached to the robot");
        return false;
      }
    }
    // If the suction voxel is attached to the robot in the planning scene but shouldn't be
    else if (!suction_voxel_enabled[voxel_ix] && aco_exists)
    {
      // Mark object to be removed
      suction_voxel_co.operation = moveit_msgs::CollisionObject::REMOVE;

      // Dettach the collision object
      if (!planning_scene->processAttachedCollisionObjectMsg(suction_voxel_aco))
      {
        ROS_WARN_STREAM_NAMED(logger_name,
                              "Failed to processAttachedCollisionObjectMsg for: " << suction_voxel_aco.object.id);
        return false;
      }

      // Check if the collision object was successfully detached
      if (planning_scene->getAttachedCollisionObjectMsg(aco, suction_voxel_aco.object.id))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Failed to detach object: " << suction_voxel_aco.object.id);
        ROS_DEBUG_STREAM_NAMED(logger_name, "Found : \n" << aco);
        return false;
      }

      // Remove Collision object from planning scene
      if (!planning_scene->processCollisionObjectMsg(suction_voxel_co))
      {
        ROS_WARN_STREAM_NAMED(logger_name,
                              "Failed to process collision object msg for: " << suction_voxel_aco.object.id);
        return false;
      }

      // Check to make sure the object is no longer in the planning scene
      moveit_msgs::CollisionObject co;
      if (planning_scene->getCollisionObjectMsg(co, suction_voxel_co.id))
      {
        ROS_WARN_STREAM_NAMED(logger_name, "Failed to remove object " << suction_voxel_co.id << " from planning scene");
        ROS_DEBUG_STREAM_NAMED(logger_name, "Found : \n" << co);
        return false;
      }
    }
  }

  // Optional visualization for debugging
  if (false)
  {
    moveit_msgs::DisplayRobotState display_robot_state_msg;
    robot_state::robotStateToRobotStateMsg(planning_scene->getCurrentState(), display_robot_state_msg.state, true);
    visual_tools_->publishRobotState(display_robot_state_msg);
    visual_tools_->trigger();
    ros::Duration(0.05).sleep();
    if (false)
      visual_tools_->prompt("Displaying suction cups as collision box. 'next' to continue");
  }

  return true;
}

void SuctionGraspFilter::setSuctionVoxelOverlapCutoff(double cutoff)
{
  suction_voxel_overlap_cutoff_ = cutoff;
}

}  // namespace moveit_grasps
