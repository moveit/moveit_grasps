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
   Desc:   Filters suction grasps based on kinematic feasibility and collision
*/

#include <moveit_grasps/suction_grasp_candidate.h>

namespace moveit_grasps
{
SuctionGraspCandidate::SuctionGraspCandidate(const moveit_msgs::Grasp& grasp, const SuctionGraspDataPtr& grasp_data,
                                             const Eigen::Isometry3d& cuboid_pose)
  : GraspCandidate::GraspCandidate(grasp, std::dynamic_pointer_cast<GraspData>(grasp_data), cuboid_pose)
  , suction_voxel_overlap_(grasp_data->suction_voxel_matrix_->getNumVoxels())
{
}

void SuctionGraspCandidate::setSuctionVoxelOverlap(std::vector<double> suction_voxel_overlap)
{
  suction_voxel_overlap_ = suction_voxel_overlap;
}

std::vector<double> SuctionGraspCandidate::getSuctionVoxelOverlap()
{
  return suction_voxel_overlap_;
}

std::vector<bool> SuctionGraspCandidate::getSuctionVoxelEnabled(double suction_voxel_cutoff)
{
  std::vector<bool> suction_voxel_enabled(suction_voxel_overlap_.size());
  for (std::size_t voxel_ix = 0; voxel_ix < suction_voxel_enabled.size(); ++voxel_ix)
    suction_voxel_enabled[voxel_ix] = suction_voxel_overlap_[voxel_ix] >= suction_voxel_cutoff;
  return suction_voxel_enabled;
}

}  // namespace moveit_grasps
