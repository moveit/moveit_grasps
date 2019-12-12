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
   Desc:   Contains collected data for each potential suction grasp after it has been verified / filtered
*/

#ifndef MOVEIT_GRASPS__SUCTION_GRASP_CANDIDATE_
#define MOVEIT_GRASPS__SUCTION_GRASP_CANDIDATE_

// Parent class
#include <moveit_grasps/grasp_candidate.h>
#include <moveit_grasps/suction_grasp_data.h>

namespace moveit_grasps
{
struct SuctionGraspFilterCode : public GraspFilterCode
{
  enum
  {
    GRASP_FILTERED_BY_SUCTION_VOXEL_OVERLAP = LAST + 1,  // No suction voxel is in sufficient contact with the target
  };
};

/**
 * \brief Contains collected data for each potential grasp after it has been verified / filtered
 *        This includes the pregrasp and grasp IK solution
 */
class SuctionGraspCandidate : public GraspCandidate
{
public:
  SuctionGraspCandidate(const moveit_msgs::Grasp& grasp, const SuctionGraspDataPtr& grasp_data,
                        const Eigen::Isometry3d& cuboid_pose);

  void setSuctionVoxelOverlap(std::vector<double> suction_voxel_overlap);

  std::vector<double> getSuctionVoxelOverlap();

  std::vector<bool> getSuctionVoxelEnabled(double cutoff);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  // A vector of fractions maped to suction gripper voxels. [0,1] representing the fraction of the
  // suction voxel that overlaps the object
  std::vector<double> suction_voxel_overlap_;

};  // class

typedef std::shared_ptr<SuctionGraspCandidate> SuctionGraspCandidatePtr;

}  // namespace moveit_grasps
#endif
