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
  Eigen::Vector3d scores = -Eigen::Vector3d(grasp_pose.translation() - ideal_pose.translation()).array().abs();

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

Eigen::Vector2d GraspScorer::scoreGraspOverhang(const Eigen::Affine3d& grasp_pose, const GraspDataPtr& grasp_data,
                                                const Eigen::Affine3d& object_pose, const Eigen::Vector3d& object_size,
                                                moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
{
  Eigen::Vector2d scores(0, 0);

  // We use 2D since we only care about x and y
  Eigen::Vector2d gripper_corner_tr(grasp_data->active_suction_range_x_ / 2.0,
                                    grasp_data->active_suction_range_y_ / 2.0);
  Eigen::Vector2d gripper_corner_tl(-grasp_data->active_suction_range_x_ / 2.0,
                                    grasp_data->active_suction_range_y_ / 2.0);
  Eigen::Vector2d gripper_corner_br(grasp_data->active_suction_range_x_ / 2.0,
                                    -grasp_data->active_suction_range_y_ / 2.0);
  Eigen::Vector2d gripper_corner_bl(-grasp_data->active_suction_range_x_ / 2.0,
                                    -grasp_data->active_suction_range_y_ / 2.0);

  Eigen::Affine3d object_to_gripper_transform = object_pose.inverse() * grasp_pose;
  Eigen::Affine2d object_to_gripper_transform_2d =
      Eigen::Translation2d(object_to_gripper_transform.translation().topRows<2>()) *
      object_to_gripper_transform.linear().topLeftCorner<2, 2>();

  gripper_corner_tr = object_to_gripper_transform_2d * gripper_corner_tr;
  gripper_corner_tl = object_to_gripper_transform_2d * gripper_corner_tl;
  gripper_corner_br = object_to_gripper_transform_2d * gripper_corner_br;
  gripper_corner_bl = object_to_gripper_transform_2d * gripper_corner_bl;

  double gripper_max_x =
      std::max({ gripper_corner_tr.x(), gripper_corner_tl.x(), gripper_corner_br.x(), gripper_corner_bl.x() });
  double gripper_min_x =
      std::min({ gripper_corner_tr.x(), gripper_corner_tl.x(), gripper_corner_br.x(), gripper_corner_bl.x() });
  double gripper_max_y =
      std::max({ gripper_corner_tr.y(), gripper_corner_tl.y(), gripper_corner_br.y(), gripper_corner_bl.y() });
  double gripper_min_y =
      std::min({ gripper_corner_tr.y(), gripper_corner_tl.y(), gripper_corner_br.y(), gripper_corner_bl.y() });

  double box_max_x = object_size[0] / 2.0;
  double box_min_x = -object_size[0] / 2.0;
  double box_max_y = object_size[1] / 2.0;
  double box_min_y = -object_size[1] / 2.0;

  if (gripper_max_x > box_max_x)
    scores[0] -= gripper_max_x - box_max_x;

  if (gripper_min_x < box_min_x)
    scores[0] -= box_min_x - gripper_min_x;

  if (gripper_max_y > box_max_y)
    scores[1] -= gripper_max_y - box_max_y;

  if (gripper_min_y < box_min_y)
    scores[1] -= box_min_y - gripper_min_y;

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.overhang", "\n"
                                                      << "\n\t max x gripper:  " << gripper_max_x
                                                      << "\n\t min x gripper:  " << gripper_min_x
                                                      << "\n\t max y gripper:  " << gripper_max_y
                                                      << "\n\t min y gripper:  " << gripper_min_y
                                                      << "\n\t max x box    :  " << box_max_x << "\n\t min x box    :  "
                                                      << box_min_x << "\n\t max y box    :  " << box_max_y
                                                      << "\n\t min y box    :  " << box_min_y
                                                      << "\n\t x score      :  " << scores[0]
                                                      << "\n\t y score      :  " << scores[1] << std::endl);

  if (visual_tools)
  {
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();
    Eigen::Affine3d gripper_corner_tr_3d = Eigen::Affine3d::Identity();
    Eigen::Affine3d gripper_corner_tl_3d = Eigen::Affine3d::Identity();
    Eigen::Affine3d gripper_corner_br_3d = Eigen::Affine3d::Identity();
    Eigen::Affine3d gripper_corner_bl_3d = Eigen::Affine3d::Identity();
    gripper_corner_tr_3d.translation() = Eigen::Vector3d(gripper_corner_tr.x(), gripper_corner_tr.y(), 2.0);
    gripper_corner_tl_3d.translation() = Eigen::Vector3d(gripper_corner_tl.x(), gripper_corner_tl.y(), 2.0);
    gripper_corner_br_3d.translation() = Eigen::Vector3d(gripper_corner_br.x(), gripper_corner_br.y(), 2.0);
    gripper_corner_bl_3d.translation() = Eigen::Vector3d(gripper_corner_bl.x(), gripper_corner_bl.y(), 2.0);

    Eigen::Affine3d box_corner_tr = Eigen::Affine3d::Identity();
    Eigen::Affine3d box_corner_tl = Eigen::Affine3d::Identity();
    Eigen::Affine3d box_corner_br = Eigen::Affine3d::Identity();
    Eigen::Affine3d box_corner_bl = Eigen::Affine3d::Identity();
    box_corner_tr.translation() = Eigen::Vector3d(object_size[0] / 2.0, object_size[1] / 2.0, 2.0);
    box_corner_tl.translation() = Eigen::Vector3d(object_size[0] / 2.0, -object_size[1] / 2.0, 2.0);
    box_corner_br.translation() = Eigen::Vector3d(-object_size[0] / 2.0, object_size[1] / 2.0, 2.0);
    box_corner_bl.translation() = Eigen::Vector3d(-object_size[0] / 2.0, -object_size[1] / 2.0, 2.0);

    visual_tools->publishAxisLabeled(gripper_corner_tr_3d, "gripper_tr", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(gripper_corner_tl_3d, "gripper_tl", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(gripper_corner_br_3d, "gripper_br", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(gripper_corner_bl_3d, "gripper_bl", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(box_corner_tr, "box_tr", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(box_corner_tl, "box_tl", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(box_corner_br, "box_br", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(box_corner_bl, "box_bl", rviz_visual_tools::SMALL);
    visual_tools->trigger();
    visual_tools->prompt("continue?");
  }

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
