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
double GraspScoreWeights::computeScore(const Eigen::Vector3d& orientation_scores,
                                       const Eigen::Vector3d& translation_scores, bool verbose) const
{
  double total_score =
      orientation_scores[0] * orientation_x_score_weight_ + orientation_scores[1] * orientation_y_score_weight_ +
      orientation_scores[2] * orientation_z_score_weight_ + translation_scores[0] * translation_x_score_weight_ +
      translation_scores[1] * translation_y_score_weight_ + translation_scores[2] * translation_z_score_weight_;

  total_score /= getWeightTotal();

  if (verbose)
  {
    static const std::string logger_name = "grasp_scorer.compute_score";
    ROS_DEBUG_STREAM_NAMED(logger_name, "Grasp score: ");
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(logger_name, "\torientation_score.x = " << orientation_scores[0] << "\tweight = "<< orientation_x_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\torientation_score.y = " << orientation_scores[1] << "\tweight = "<< orientation_y_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\torientation_score.z = " << orientation_scores[2] << "\tweight = "<< orientation_z_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttranslation_score.x = " << translation_scores[0] << "\tweight = "<< translation_x_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttranslation_score.y = " << translation_scores[1] << "\tweight = "<< translation_y_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttranslation_score.z = " << translation_scores[2] << "\tweight = "<< translation_z_score_weight_);
    // Total
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttotal_score = " << total_score);
    // clang-format on
  }
  return total_score;
}

double GraspScoreWeights::getWeightTotal() const
{
  return orientation_x_score_weight_ + orientation_y_score_weight_ + orientation_z_score_weight_ +
         translation_x_score_weight_ + translation_y_score_weight_ + translation_z_score_weight_;
}

Eigen::Vector3d GraspScorer::scoreRotationsFromDesired(const Eigen::Isometry3d& grasp_pose_tcp,
                                                       const Eigen::Isometry3d& ideal_pose)
{
  Eigen::Vector3d grasp_pose_axis;
  Eigen::Vector3d ideal_pose_axis;
  Eigen::Vector3d scores;
  double cos_angle;
  double angle;

  // get angle between x-axes
  grasp_pose_axis = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitX();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitX();
  cos_angle = grasp_pose_axis.dot(ideal_pose_axis);
  angle = acos(std::max(-1.0, std::min(1.0, cos_angle)));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "x angle = " << angle * 180.0 / M_PI);
  scores[0] = (M_PI - angle) / M_PI;

  // get angle between y-axes
  grasp_pose_axis = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitY();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitY();
  cos_angle = grasp_pose_axis.dot(ideal_pose_axis);
  angle = acos(std::max(-1.0, std::min(1.0, cos_angle)));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "y angle = " << angle * 180.0 / M_PI);
  scores[1] = (M_PI - angle) / M_PI;

  // get angle between z-axes
  grasp_pose_axis = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitZ();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitZ();
  cos_angle = grasp_pose_axis.dot(ideal_pose_axis);
  angle = acos(std::max(-1.0, std::min(1.0, cos_angle)));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "z angle = " << angle * 180.0 / M_PI);
  scores[2] = (M_PI - angle) / M_PI;

  return scores;
}

Eigen::Vector3d GraspScorer::scoreGraspTranslation(const Eigen::Isometry3d& grasp_pose_tcp,
                                                   const Eigen::Isometry3d& ideal_pose)
{
  // We assume that the ideal is in the middle
  Eigen::Vector3d scores = -Eigen::Vector3d(grasp_pose_tcp.translation() - ideal_pose.translation()).array().abs();

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.scoreGraspTranslation",
                         "value, ideal, score:\n"
                             << "x: " << grasp_pose_tcp.translation()[0] << "\t" << ideal_pose.translation()[0] << "\t"
                             << scores[0] << "\n"
                             << "y: " << grasp_pose_tcp.translation()[1] << "\t" << ideal_pose.translation()[1] << "\t"
                             << scores[1] << "\n"
                             << "x: " << grasp_pose_tcp.translation()[2] << "\t" << ideal_pose.translation()[2] << "\t"
                             << scores[2] << "\n");

  return scores;
}

Eigen::Vector3d GraspScorer::scoreGraspTranslation(const Eigen::Isometry3d& grasp_pose_tcp,
                                                   const Eigen::Vector3d& min_translations,
                                                   const Eigen::Vector3d& max_translations)
{
  Eigen::Vector3d scores;

  for (std::size_t i = 0; i < 3; ++i)
  {
    // We assume that the ideal is in the middle
    double ideal = (max_translations[i] + min_translations[i]) / 2;
    double translation = grasp_pose_tcp.translation()[i] - ideal;
    double range = max_translations[i] - min_translations[i];
    double score;
    if (range == 0)
      score = 0;
    else
      score = translation / range;

    scores[i] = pow(score, 2);
  }

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.translation",
                         "\nvalue, min, max, score:\n"
                             << grasp_pose_tcp.translation()[0] << ", " << min_translations[0] << ", "
                             << max_translations[0] << ", " << scores[0] << "\n"
                             << grasp_pose_tcp.translation()[1] << ", " << min_translations[1] << ", "
                             << max_translations[1] << ", " << scores[1] << "\n"
                             << grasp_pose_tcp.translation()[2] << ", " << min_translations[2] << ", "
                             << max_translations[2] << ", " << scores[2] << "\n");

  return scores;
}

}  // namespace moveit_grasps
