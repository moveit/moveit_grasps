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
 * Authors : Andy McEvoy ( mcevoy.andy@gmail.com )
 * Desc    : Functions for scoring grasps. See *.h file for documentation
 */

#include <moveit_grasps/grasp_scorer.h>

namespace moveit_grasps
{
double GraspScorer::scoreGraspWidth(const GraspDataPtr grasp_data, double percent_open)
{
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.graspWidth", "raw score = " << percent_open);
  return pow(percent_open, 2);
}

double GraspScorer::scoreDistanceToPalm(const Eigen::Affine3d& grasp_pose, const GraspDataPtr grasp_data,
                                        const Eigen::Affine3d& object_pose, const double& min_grasp_distance,
                                        const double& max_grasp_distance)
{
  // DEV NOTE: grasp_data is not used but should be. See *.h for explaination.

  double distance = (grasp_pose.translation() - object_pose.translation()).norm();
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.distance", "distance = " << distance << ", " << min_grasp_distance << ":"
                                                                << max_grasp_distance);

  double score = 1.0 - (distance - min_grasp_distance) / (max_grasp_distance - min_grasp_distance);

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.distance", "raw score = " << score);
  if (score < 0)
    ROS_WARN_STREAM_NAMED("grasp_scorer.distance", "score < 0!");
  return pow(score, 4);
}

Eigen::Vector3d GraspScorer::scoreGraspTranslation(const Eigen::Affine3d& grasp_pose,
                                                   const Eigen::Vector3d& min_translations,
                                                   const Eigen::Vector3d& max_translations)
{
  Eigen::Vector3d scores;

  for (std::size_t i = 0; i < 3; i++)
  {
    double translation = grasp_pose.translation()[i] - min_translations[i];
    double delta = max_translations[i] - min_translations[i];
    double score = translation / delta;

    scores[i] = pow(score, 2);
  }

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.translation", "value, min, max, score:\n"
                                                         << grasp_pose.translation()[0] << ", " << min_translations[0]
                                                         << ", " << max_translations[0] << ", " << scores[0] << "\n"
                                                         << grasp_pose.translation()[1] << ", " << min_translations[1]
                                                         << ", " << max_translations[1] << ", " << scores[1] << "\n"
                                                         << grasp_pose.translation()[2] << ", " << min_translations[2]
                                                         << ", " << max_translations[2] << ", " << scores[2] << "\n");

  return scores;
}

Eigen::Vector3d GraspScorer::scoreRotationsFromDesired(const Eigen::Affine3d& grasp_pose,
                                                       const Eigen::Affine3d& ideal_pose)
{
  Eigen::Vector3d grasp_pose_axis;
  Eigen::Vector3d ideal_pose_axis;
  Eigen::Vector3d scores;
  double angle;

  // get angle between x-axes
  grasp_pose_axis = grasp_pose.rotation() * Eigen::Vector3d::UnitX();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitX();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "x angle = " << angle * 180.0 / M_PI);
  scores[0] = (M_PI - angle) / M_PI;
  // scores[0] = pow(scores[0],2);

  // get angle between y-axes
  grasp_pose_axis = grasp_pose.rotation() * Eigen::Vector3d::UnitY();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitY();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "y angle = " << angle * 180.0 / M_PI);
  scores[1] = (M_PI - angle) / M_PI;
  // scores[1] = pow(scores[1],2);

  // get angle between z-axes
  grasp_pose_axis = grasp_pose.rotation() * Eigen::Vector3d::UnitZ();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitZ();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "z angle = " << angle * 180.0 / M_PI);
  scores[2] = (M_PI - angle) / M_PI;
  // scores[2] = pow(scores[2],2);

  return scores;
}

}  // end namespace moveit_grasps
