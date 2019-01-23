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
  // TODO(mcevoyandy): grasp_data is not used but should be. See *.h for explaination.

  double distance = (grasp_pose.translation() - object_pose.translation()).norm();
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.distance", "distance = " << distance << ", " << min_grasp_distance << ":"
                                                                << max_grasp_distance);

  double score = 1.0 - (distance - min_grasp_distance) / (max_grasp_distance - min_grasp_distance);

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.distance", "raw score = " << score);
  if (score < 0)
    ROS_WARN_STREAM_NAMED("grasp_scorer.distance", "score < 0!");
  return pow(score, 4);
}

Eigen::Vector3d GraspScorer::scoreGraspTranslation(const Eigen::Affine3d& grasp_pose, const Eigen::Affine3d& ideal_pose)
{
  // We assume that the ideal is in the middle
  Eigen::Vector3d scores = -Eigen::Vector3d(grasp_pose.translation() - ideal_pose.translation()).array();

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.scoreGraspTranslation",
                         "value, ideal, score:\n"
                             << "x: " << grasp_pose.translation()[0] << "\t" << ideal_pose.translation()[0] << "\t"
                             << scores[0] << "\n"
                             << "y: " << grasp_pose.translation()[1] << "\t" << ideal_pose.translation()[1] << "\t"
                             << scores[1] << "\n"
                             << "x: " << grasp_pose.translation()[2] << "\t" << ideal_pose.translation()[2] << "\t"
                             << scores[2] << "\n");

  return scores;
}

Eigen::Vector3d GraspScorer::scoreGraspTranslation(const Eigen::Affine3d& grasp_pose,
                                                   const Eigen::Vector3d& min_translations,
                                                   const Eigen::Vector3d& max_translations)
{
  Eigen::Vector3d scores;

  for (std::size_t i = 0; i < 3; i++)
  {
    // We assume that the ideal is in the middle
    double ideal = (max_translations[i] + min_translations[i]) / 2;
    double translation = grasp_pose.translation()[i] - ideal;
    double range = max_translations[i] - min_translations[i];
    double score = translation / range;

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

Eigen::Vector3d GraspScorer::scoreGraspOverhang(const Eigen::Affine3d& grasp_pose,
                                                const GraspDataPtr& grasp_data,
                                                const Eigen::Affine3d& object_pose,
                                                const Eigen::Vector3d& object_size)
{
  Eigen::Vector3d scores(0, 0, 0);


  Eigen::Affine3d gripper_corner_tr = Eigen::Affine3d::Identity();
  Eigen::Affine3d gripper_corner_tl = Eigen::Affine3d::Identity();
  Eigen::Affine3d gripper_corner_br = Eigen::Affine3d::Identity();
  Eigen::Affine3d gripper_corner_bl = Eigen::Affine3d::Identity();

  Eigen::Affine3d box_corner_tr = Eigen::Affine3d::Identity();
  Eigen::Affine3d box_corner_tl = Eigen::Affine3d::Identity();
  Eigen::Affine3d box_corner_br = Eigen::Affine3d::Identity();
  Eigen::Affine3d box_corner_bl = Eigen::Affine3d::Identity();

  gripper_corner_tr.translation() = Eigen::Vector3d( grasp_data->active_suction_range_x_ / 2.0,  grasp_data->active_suction_range_y_ / 2.0, 0.0);
  gripper_corner_tl.translation() = Eigen::Vector3d( grasp_data->active_suction_range_x_ / 2.0, -grasp_data->active_suction_range_y_ / 2.0, 0.0);
  gripper_corner_br.translation() = Eigen::Vector3d(-grasp_data->active_suction_range_x_ / 2.0,  grasp_data->active_suction_range_y_ / 2.0, 0.0);
  gripper_corner_bl.translation() = Eigen::Vector3d(-grasp_data->active_suction_range_x_ / 2.0, -grasp_data->active_suction_range_y_ / 2.0, 0.0);
  box_corner_tr.translation() = Eigen::Vector3d( object_size[0] / 2.0,  object_size[1] / 2.0, 0.0);
  box_corner_tl.translation() = Eigen::Vector3d( object_size[0] / 2.0, -object_size[1] / 2.0, 0.0);
  box_corner_br.translation() = Eigen::Vector3d(-object_size[0] / 2.0,  object_size[1] / 2.0, 0.0);
  box_corner_bl.translation() = Eigen::Vector3d(-object_size[0] / 2.0, -object_size[1] / 2.0, 0.0);

  gripper_corner_tr = object_pose.inverse() * grasp_pose * gripper_corner_tr;
  gripper_corner_tl = object_pose.inverse() * grasp_pose * gripper_corner_tl;
  gripper_corner_br = object_pose.inverse() * grasp_pose * gripper_corner_br;
  gripper_corner_bl = object_pose.inverse() * grasp_pose * gripper_corner_bl;

  // bool debug_overhang = true;
  // if (debug_overhang)
  // {
  //   box_corner_tr = object_pose * box_corner_tr;
  //   box_corner_tl = object_pose * box_corner_tl;
  //   box_corner_br = object_pose * box_corner_br;
  //   box_corner_bl = object_pose * box_corner_bl;
  //   gripper_corner_tr = object_pose * gripper_corner_tr;
  //   gripper_corner_tl = object_pose * gripper_corner_tl;
  //   gripper_corner_br = object_pose * gripper_corner_br;
  //   gripper_corner_bl = object_pose * gripper_corner_bl;
  //   visual_tools->publishAxisLabeled(gripper_corner_tr, "gripper_corner_tr", rviz_visual_tools::SMALL);
  //   visual_tools->publishAxisLabeled(gripper_corner_tl, "gripper_corner_tl", rviz_visual_tools::SMALL);
  //   visual_tools->publishAxisLabeled(gripper_corner_br, "gripper_corner_br", rviz_visual_tools::SMALL);
  //   visual_tools->publishAxisLabeled(gripper_corner_bl, "gripper_corner_bl", rviz_visual_tools::SMALL);
  //   visual_tools->publishAxisLabeled(box_corner_tr, "box_corner_tr", rviz_visual_tools::SMALL);
  //   visual_tools->publishAxisLabeled(box_corner_tl, "box_corner_tl", rviz_visual_tools::SMALL);
  //   visual_tools->publishAxisLabeled(box_corner_br, "box_corner_br", rviz_visual_tools::SMALL);
  //   visual_tools->publishAxisLabeled(box_corner_bl, "box_corner_bl", rviz_visual_tools::SMALL);
  //   visual_tools->trigger();
  //   gripper_corner_tr = object_pose.inverse() * gripper_corner_tr;
  //   gripper_corner_tl = object_pose.inverse() * gripper_corner_tl;
  //   gripper_corner_br = object_pose.inverse() * gripper_corner_br;
  //   gripper_corner_bl = object_pose.inverse() * gripper_corner_bl;
  //   box_corner_tr = object_pose.inverse() * box_corner_tr;
  //   box_corner_tl = object_pose.inverse() * box_corner_tl;
  //   box_corner_br = object_pose.inverse() * box_corner_br;
  //   box_corner_bl = object_pose.inverse() * box_corner_bl;
  // }


  double box_max_tx = std::max(box_corner_tr.translation().x(), box_corner_tl.translation().x());
  double box_max_bx = std::max(box_corner_br.translation().x(), box_corner_bl.translation().x());
  double box_max_x  = std::max(box_max_tx, box_max_bx);

  double box_min_tx = std::min(box_corner_tr.translation().x(), box_corner_tl.translation().x());
  double box_min_bx = std::min(box_corner_br.translation().x(), box_corner_bl.translation().x());
  double box_min_x  = std::min(box_min_tx, box_min_bx);

  double box_max_ty = std::max(box_corner_tr.translation().y(), box_corner_tl.translation().y());
  double box_max_by = std::max(box_corner_br.translation().y(), box_corner_bl.translation().y());
  double box_max_y  = std::max(box_max_ty, box_max_by);

  double box_min_ty = std::min(box_corner_tr.translation().y(), box_corner_tl.translation().y());
  double box_min_by = std::min(box_corner_br.translation().y(), box_corner_bl.translation().y());
  double box_min_y  = std::min(box_min_ty, box_min_by);


  double gripper_max_tx = std::max(gripper_corner_tr.translation().x(), gripper_corner_tl.translation().x());
  double gripper_max_bx = std::max(gripper_corner_br.translation().x(), gripper_corner_bl.translation().x());
  double gripper_max_x  = std::max(gripper_max_tx, gripper_max_bx);

  double gripper_min_tx = std::min(gripper_corner_tr.translation().x(), gripper_corner_tl.translation().x());
  double gripper_min_bx = std::min(gripper_corner_br.translation().x(), gripper_corner_bl.translation().x());
  double gripper_min_x  = std::min(gripper_min_tx, gripper_min_bx);

  double gripper_max_ty = std::max(gripper_corner_tr.translation().y(), gripper_corner_tl.translation().y());
  double gripper_max_by = std::max(gripper_corner_br.translation().y(), gripper_corner_bl.translation().y());
  double gripper_max_y  = std::max(gripper_max_ty, gripper_max_by);

  double gripper_min_ty = std::min(gripper_corner_tr.translation().y(), gripper_corner_tl.translation().y());
  double gripper_min_by = std::min(gripper_corner_br.translation().y(), gripper_corner_bl.translation().y());
  double gripper_min_y  = std::min(gripper_min_ty, gripper_min_by);


  if(gripper_max_x > box_max_x)
    scores[0] -= gripper_max_x - box_max_x;

  if(gripper_min_x < box_min_x)
    scores[0] -= box_min_x - gripper_min_x;

  if(gripper_max_y > box_max_y)
    scores[1] -= gripper_max_y - box_max_y;

  if(gripper_min_y < box_min_y)
    scores[1] -= box_min_y - gripper_min_y;

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.overhang", "" << scores[0] << "\t" << scores[1]);

  // if (debug_overhang)
  // {
  //   visual_tools->prompt("continue?");
  //   visual_tools->deleteAllMarkers();
  //   visual_tools->resetMarkerCounts();
  //   visual_tools->trigger();
  // }

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

  // get angle between y-axes
  grasp_pose_axis = grasp_pose.rotation() * Eigen::Vector3d::UnitY();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitY();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "y angle = " << angle * 180.0 / M_PI);
  scores[1] = (M_PI - angle) / M_PI;

  // get angle between z-axes
  grasp_pose_axis = grasp_pose.rotation() * Eigen::Vector3d::UnitZ();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitZ();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "z angle = " << angle * 180.0 / M_PI);
  scores[2] = (M_PI - angle) / M_PI;

  return scores;
}

}  // end namespace moveit_grasps
