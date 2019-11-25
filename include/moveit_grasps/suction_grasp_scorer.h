/*********************************************************************
 * Software License Agreement ("Modified BSD License")
 *
 * Copyright (c) 2014, University of Colorado, Boulder
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * * Neither the name of the Univ of CO, Boulder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/**
 * Authors : Andy McEvoy
 * Desc    : Functions for scoring generated grasps
 */

#ifndef MOVEIT_GRASPS__SUCTION_GRASP_SCORER_
#define MOVEIT_GRASPS__SUCTION_GRASP_SCORER_

#include <moveit_grasps/suction_grasp_data.h>
#include <moveit_grasps/grasp_scorer.h>

namespace moveit_grasps
{
struct SuctionGraspScoreWeights : public GraspScoreWeights
{
  SuctionGraspScoreWeights() : GraspScoreWeights(), overhang_score_weight_(1.0)
  {
  }

  /* \brief Compute the weighted score given the orientation and translation scores */
  double computeScore(const Eigen::Vector3d& orientation_scores, const Eigen::Vector3d& translation_scores,
                      double overhang_score, bool verbose = false) const;

  /* \brief returns the sum of the grasp score weights*/
  double getWeightTotal() const override;

  // Suction gripper specific weights
  double overhang_score_weight_;
};
// Create smart pointers for this class
typedef std::shared_ptr<SuctionGraspScoreWeights> SuctionGraspScoreWeightsPtr;
typedef std::shared_ptr<const SuctionGraspScoreWeights> SuctionGraspScoreWeightsConstPtr;

class SuctionGraspScorer : public GraspScorer
{
public:
  /**
   * \brief Score a suction grasp based on the overlap between each voxel and the object.
   * \param grasp_pose_tcp - the pose of the end effector (not the eef mount)
   * \param object_pose - the pose of the object being grasped
   * \param object_size - the size of the object represented as a vector [x,y,z]
   * \param visual_tools - set to a moveit_visual_tools pointer to enable visual debugging
   * \param overlap_vector - Populates with a vector of fractions. Each value represents the i'th voxel's fractional
   * overlap
   * \return double - a score. The sum of the squares of the fractions
   */
  static double scoreSuctionVoxelOverlap(const Eigen::Isometry3d& grasp_pose_tcp, const SuctionGraspDataPtr& grasp_data,
                                         const Eigen::Isometry3d& object_pose, const Eigen::Vector3d& object_size,
                                         std::vector<double>& overlap_vector,
                                         moveit_visual_tools::MoveItVisualToolsPtr visual_tools = nullptr);
};

}  // namespace moveit_grasps

#endif
