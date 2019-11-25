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

#ifndef MOVEIT_GRASPS__TWO_FINGER_GRASP_SCORER_
#define MOVEIT_GRASPS__TWO_FINGER_GRASP_SCORER_

#include <moveit_grasps/two_finger_grasp_data.h>
#include <moveit_grasps/grasp_scorer.h>

namespace moveit_grasps
{
struct TwoFingerGraspScoreWeights : public GraspScoreWeights
{
  TwoFingerGraspScoreWeights() : GraspScoreWeights(), depth_score_weight_(1.0), width_score_weight_(1.0)
  {
  }

  /* \brief Compute the weighted score given the orientation and translation scores */
  double computeScore(const Eigen::Vector3d& orientation_scores, const Eigen::Vector3d& translation_scores,
                      double depth_score, double width_score, bool verbose = false) const;

  /* \brief returns the sum of the grasp score weights*/
  double getWeightTotal() const override;

  // Finger gripper specific weights
  double depth_score_weight_;
  double width_score_weight_;
};
// Create smart pointers for this class
typedef std::shared_ptr<TwoFingerGraspScoreWeights> TwoFingerGraspScoreWeightsPtr;

class TwoFingerGraspScorer : public GraspScorer
{
public:
  /**
   * \brief Scores the grasp on how wide the fingers are on approach, the more open the better
   * \param grasp_data - pointer to grasp info
   * \param percent_open - amount the gripper is open
   *                       0.0 -> gripper is open to the object width + minimum padding
   *                       1.0 -> gripper is in full open position
   * \return the unweighted score:
   *         1.0 -> gripper is wide open,
   *         0.0 -> gripper is at minimum position.
   */
  static double scoreGraspWidth(const TwoFingerGraspDataPtr& grasp_data, double percent_open);

  /**
   * \brief Score the grasp based on how far the object is from the palm of the hand
   * \param grasp_pose_tcpgrasp_pose_tcp - the pose of the end effector (not the eef mount)
   * \param grasp_data - pointer to grasp info
   * \param object_pose - the pose of the object being grasped
   * \param max_grasp_distance - the maximum acceptable distance from palm
   * \return the unweighted score:
   *         1.0 -> object pose and grasp pose have same translation values
   *         0.0 -> object is at max distanct
   *       < 0.0 -> object is beyond the max_grasp_distance
   */
  // DEV NOTE: when this function is called we've lost the references to the acutal size of the object.
  // max_distance should be the length of the fingers minus some minimum amount that the fingers need to grip an object
  // since we don't know the distance from the centoid of the object to the edge of the object, this is set as an
  // arbitrary number given our target object set (i.e. I based it off of the cheese it box)
  static double scoreDistanceToPalm(const Eigen::Isometry3d& grasp_pose_tcp, const TwoFingerGraspDataPtr& grasp_data,
                                    const Eigen::Isometry3d& object_pose, double min_grasp_distance,
                                    double max_grasp_distance);
};

}  // namespace moveit_grasps

#endif
