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

/* Author: Mike Lautman <mike@picknik.ai>
   Desc:   Filters grasps based on kinematic feasibility and collision
*/

#ifndef MOVEIT_GRASPS__SUCTION_GRASP_FILTER_
#define MOVEIT_GRASPS__SUCTION_GRASP_FILTER_

// Grasping
#include <moveit_grasps/suction_grasp_generator.h>

// Parent class
#include <moveit_grasps/grasp_filter.h>

namespace moveit_grasps
{
class SuctionGraspFilter : public GraspFilter
{
public:
  // Constructor
  SuctionGraspFilter(const robot_state::RobotStatePtr& robot_state,
                     const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools);

  /**
   * \brief Return grasps that are kinematically feasible
   * \param grasp_candidates - all possible grasps that this will test. this vector is returned modified
   * \param arm_jmg - the arm to solve the IK problem on
   * \param filter_pregrasp -whether to also check ik feasibility for the pregrasp position
   * \param visualize - visualize IK filtering
   * \return number of grasps remaining
   */
  std::size_t filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                 const planning_scene::PlanningScenePtr& planning_scene_monitor,
                                 const robot_model::JointModelGroup* arm_jmg,
                                 const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp, bool visualize,
                                 const std::string& target_object_id = "") override;

  /**
   * \brief Print grasp filtering statistics
   */
  void printFilterStatistics(const std::vector<GraspCandidatePtr>& grasp_candidates) const override;

  /**
   * \brief Thread for checking part of the possible grasps list
   */
  bool processCandidateGrasp(const IkThreadStructPtr& ik_thread_struct) override;

  /**
   * \brief Filter grasps that do not have a valid suction voxel overlap
   * \param grasp_candidates - all possible grasps that this will test. this vector is returned modified
   */
  bool filterGraspsBySuctionVoxelOverlap(std::vector<GraspCandidatePtr>& grasp_candidates);

  [[deprecated("Use filterGraspsBySuctionVoxelOverlap")]] bool
  filterGraspsBySuctionVoxelOverlapCutoff(std::vector<GraspCandidatePtr>& grasp_candidates)
  {
    return filterGraspsBySuctionVoxelOverlap(grasp_candidates);
  }

  /**
   * \brief  For suction grippers, set the cutoff threshold used by preFilterBySuctionVoxelOverlap to
   *         pre filter grasps.
   * \param cutoff - A fractional cutoff between (0, 1] where at least one voxel must at least that much
   *                 of it's surface overlaping with the target object
   */
  void setSuctionVoxelOverlapCutoff(double cutoff);

protected:
  /**
   * \brief  Remove all attached collision objects representing suction voxels from the planning scene.
   * \param grasp_data - A pointer to a SuctionGraspData with a populated SuctionVoxelMatrix
   * \param planning_scene - A pointer to a planning scene where we will attach the collision objects
   * \return true if all suction voxel attached collision objects removed
   */
  bool removeAllSuctionCupCO(const SuctionGraspDataPtr& grasp_data,
                             const planning_scene::PlanningScenePtr& planning_scene);

  /**
   * \brief  Add a collision object in the location of every active suction voxel. This is used for collision
   *         checking between the suction cups and other objects in the scene that you do not want to pick up
   * \param grasp_data - A pointer to a SuctionGraspData with a populated SuctionVoxelMatrix
   * \param planning_scene - A pointer to a planning scene where we will attach the collision objects
   * \param collision_object_names - Output, the collision object names added to the planning scene
   */
  bool attachActiveSuctionCupCO(const SuctionGraspDataPtr& grasp_data, const std::vector<bool>& suction_voxel_enabled,
                                const planning_scene::PlanningScenePtr& planning_scene,
                                std::vector<std::string>& collision_object_names);

  /* \brief a method for transforming from voxel index to voxel collision object ID used by attachActiveSuctionCupCO and
   * removeAllSuctionCupCO */
  static std::string suctionVoxelIxToCollisionObjectId(std::size_t voxel_ix);

private:
  // Name for logging
  const std::string name_;

  // A cutoff threshold [0,1] where at least one suction voxe must have more than this fraction overlap
  // with the target object
  double suction_voxel_overlap_cutoff_;

};  // end of class

typedef std::shared_ptr<SuctionGraspFilter> SuctionGraspFilterPtr;
typedef std::shared_ptr<const SuctionGraspFilter> SuctionGraspFilterConstPtr;

}  // namespace moveit_grasps

#endif
