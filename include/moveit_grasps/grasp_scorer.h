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

#ifndef MOVEIT_GRASPS__GRASP_SCORER_
#define MOVEIT_GRASPS__GRASP_SCORER_

#include <cmath>

#include <ros/ros.h>

#include <moveit_grasps/grasp_data.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace moveit_grasps
{
struct GraspScoreWeights
{
  GraspScoreWeights()
    : orientation_x_score_weight_(1.0)
    , orientation_y_score_weight_(1.0)
    , orientation_z_score_weight_(1.0)
    , translation_x_score_weight_(1.0)
    , translation_y_score_weight_(1.0)
    , translation_z_score_weight_(1.0)
  {
  }

  /* \brief Compute the weighted score given the orientation and translation scores */
  double computeScore(const Eigen::Vector3d& orientation_scores, const Eigen::Vector3d& translation_scores,
                      bool verbose = false) const;

  /* \brief returns the sum of the grasp score weights*/
  virtual double getWeightTotal() const;

  double orientation_x_score_weight_;
  double orientation_y_score_weight_;
  double orientation_z_score_weight_;
  double translation_x_score_weight_;
  double translation_y_score_weight_;
  double translation_z_score_weight_;
};
// Create smart pointers for this struct
typedef std::shared_ptr<GraspScoreWeights> GraspScoreWeightsPtr;

class GraspScorer
{
public:
  /**
   * \brief Scores each axis of the grasp based on its angle to the desired pose axis.
   * \param grasp_pose_tcp - the pose of the end effector
   * \param ideal_pose - the ideal grasp pose (ex: straight into the bin)
   * \return the unweighted scores:
   *         1.0 -> 0 degrees between grasp axis and desired axis,
   *         0.0 -> 180 degrees
   */
  static Eigen::Vector3d scoreRotationsFromDesired(const Eigen::Isometry3d& grasp_pose_tcp,
                                                   const Eigen::Isometry3d& ideal_pose);

  /**
   * \brief Score the grasp based on the translation values of the grasp pose
   * \param grasp_pose_tcp - the pose of the end effector (not the eef mount)
   * \param min_translations - the minimum translation values for all grasp poses
   * \param max_translations - the maximum translation values for all grasp poses
   * \return the unweighted scores:
   *         0.0 -> pose is at the minimum translation in that axis
   *         1.0 -> pose is at the maximum translation in that axis
   */
  static Eigen::Vector3d scoreGraspTranslation(const Eigen::Isometry3d& grasp_pose_tcp,
                                               const Eigen::Vector3d& min_translations,
                                               const Eigen::Vector3d& max_translations);

  /**
   * \brief Score the grasp based on the translation values of the grasp pose
   * \param grasp_pose_tcp - the pose of the end effector (not the eef mount)
   * \param ideal_pose - the ideal pose location
   * \param object_pose - the pose of the object being grasped
   * \param object_size - the size of the object represented as a vector [x,y,z]
   * \param visual_tools - set to a moveit_visual_tools pointer to enable visual debugging
   * \return the unweighted scores:
   *         0.0 -> pose is at the ideal translation in that axis
   */
  static Eigen::Vector3d scoreGraspTranslation(const Eigen::Isometry3d& grasp_pose_tcp,
                                               const Eigen::Isometry3d& ideal_pose);
};

}  // namespace moveit_grasps

#endif
