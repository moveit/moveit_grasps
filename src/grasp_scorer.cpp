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

double GraspScorer::scoreDistanceToPalm(const Eigen::Isometry3d& grasp_pose_tcp, const GraspDataPtr grasp_data,
                                        const Eigen::Isometry3d& object_pose, const double& min_grasp_distance,
                                        const double& max_grasp_distance)
{
  // TODO(mcevoyandy): grasp_data is not used but should be. See *.h for explaination.

  double distance = (grasp_pose_tcp.translation() - object_pose.translation()).norm();
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.distance", "distance = " << distance << ", " << min_grasp_distance << ":"
                                                                << max_grasp_distance);

  double score = 1.0 - (distance - min_grasp_distance) / (max_grasp_distance - min_grasp_distance);

  ROS_DEBUG_STREAM_NAMED("grasp_scorer.distance", "raw score = " << score);
  if (score < 0)
    ROS_WARN_STREAM_NAMED("grasp_scorer.distance", "score < 0!");
  return pow(score, 4);
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

  for (std::size_t i = 0; i < 3; i++)
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

Eigen::Vector2d GraspScorer::scoreGraspOverhang(const Eigen::Isometry3d& grasp_pose_tcp, const GraspDataPtr& grasp_data,
                                                const Eigen::Isometry3d& object_pose,
                                                const Eigen::Vector3d& object_size,
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

  Eigen::Isometry3d object_to_gripper_transform = object_pose.inverse() * grasp_pose_tcp;
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
    visual_tools->prompt("continue?");
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();
    Eigen::Isometry3d gripper_corner_tr_3d = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d gripper_corner_tl_3d = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d gripper_corner_br_3d = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d gripper_corner_bl_3d = Eigen::Isometry3d::Identity();
    gripper_corner_tr_3d.translation() = Eigen::Vector3d(gripper_corner_tr.x(), gripper_corner_tr.y(), 2.0);
    gripper_corner_tl_3d.translation() = Eigen::Vector3d(gripper_corner_tl.x(), gripper_corner_tl.y(), 2.0);
    gripper_corner_br_3d.translation() = Eigen::Vector3d(gripper_corner_br.x(), gripper_corner_br.y(), 2.0);
    gripper_corner_bl_3d.translation() = Eigen::Vector3d(gripper_corner_bl.x(), gripper_corner_bl.y(), 2.0);

    Eigen::Isometry3d box_corner_tr = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d box_corner_tl = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d box_corner_br = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d box_corner_bl = Eigen::Isometry3d::Identity();
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

Eigen::Vector3d GraspScorer::scoreRotationsFromDesired(const Eigen::Isometry3d& grasp_pose_tcp,
                                                       const Eigen::Isometry3d& ideal_pose)
{
  Eigen::Vector3d grasp_pose_axis;
  Eigen::Vector3d ideal_pose_axis;
  Eigen::Vector3d scores;
  double angle;

  // get angle between x-axes
  grasp_pose_axis = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitX();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitX();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "x angle = " << angle * 180.0 / M_PI);
  scores[0] = (M_PI - angle) / M_PI;

  // get angle between y-axes
  grasp_pose_axis = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitY();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitY();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "y angle = " << angle * 180.0 / M_PI);
  scores[1] = (M_PI - angle) / M_PI;

  // get angle between z-axes
  grasp_pose_axis = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitZ();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitZ();
  angle = acos(grasp_pose_axis.dot(ideal_pose_axis));
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.angle", "z angle = " << angle * 180.0 / M_PI);
  scores[2] = (M_PI - angle) / M_PI;

  return scores;
}

std::vector<double> GraspScorer::scoreSuctionVoxelOverlap(const Eigen::Isometry3d& grasp_pose_tcp,
                                                          const GraspDataPtr& grasp_data,
                                                          const Eigen::Isometry3d& object_pose,
                                                          const Eigen::Vector3d& object_size,
                                                          moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
{
  std::vector<double> overlap_vector;
  overlap_vector.resize(grasp_data->suction_voxel_matrix_->getNumVoxels());

  Eigen::Vector3d tmp_box_bb_1 = object_pose * Eigen::Vector3d(object_size.x() / 2.0, object_size.y() / 2.0, 0);
  Eigen::Vector3d tmp_box_bb_2 = object_pose * Eigen::Vector3d(-object_size.x() / 2.0, -object_size.y() / 2.0, 0);
  Eigen::Vector3d box_bb_1 =
      Eigen::Vector3d(std::min(tmp_box_bb_1.x(), tmp_box_bb_2.x()), std::min(tmp_box_bb_1.y(), tmp_box_bb_2.y()),
                      std::max(tmp_box_bb_1.z(), tmp_box_bb_2.z()));
  Eigen::Vector3d box_bb_2 =
      Eigen::Vector3d(std::max(tmp_box_bb_1.x(), tmp_box_bb_2.x()), std::max(tmp_box_bb_1.y(), tmp_box_bb_2.y()),
                      std::min(tmp_box_bb_1.z(), tmp_box_bb_2.z()));

  if (visual_tools)
  {
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();
    visual_tools->publishAxisLabeled(grasp_pose_tcp, "tcp");
    Eigen::Isometry3d bbb1 = Eigen::Translation3d(box_bb_1) * Eigen::Quaterniond(1, 0, 0, 0);
    Eigen::Isometry3d bbb2 = Eigen::Translation3d(box_bb_2) * Eigen::Quaterniond(1, 0, 0, 0);
    visual_tools->publishAxisLabeled(bbb1, "bbb1", rviz_visual_tools::XSMALL);
    visual_tools->publishAxisLabeled(bbb2, "bbb2", rviz_visual_tools::XSMALL);
  }

  double voxel_area = grasp_data->suction_voxel_matrix_->getVoxelArea();
  for (std::size_t voxel_id = 0; voxel_id < grasp_data->suction_voxel_matrix_->getNumVoxels(); ++voxel_id)
  {
    std::shared_ptr<SuctionVoxel> voxel;
    if (!grasp_data->suction_voxel_matrix_->getSuctionVoxel(voxel_id, voxel))
    {
      ROS_ERROR_STREAM_NAMED("grasp_scorer.voxels", "voxel id: " << voxel_id << " is out of range");
      overlap_vector.clear();
      overlap_vector.resize(grasp_data->suction_voxel_matrix_->getNumVoxels());
      return overlap_vector;
    }

    Eigen::Vector3d tmp_voxel_bb_1 = grasp_pose_tcp * voxel->bottom_left_;
    Eigen::Vector3d tmp_voxel_bb_2 = grasp_pose_tcp * voxel->top_right_;
    Eigen::Vector3d voxel_bb_1 = Eigen::Vector3d(std::min(tmp_voxel_bb_1.x(), tmp_voxel_bb_2.x()),
                                                 std::min(tmp_voxel_bb_1.y(), tmp_voxel_bb_2.y()),
                                                 std::min(tmp_voxel_bb_1.z(), tmp_voxel_bb_2.z()));
    Eigen::Vector3d voxel_bb_2 = Eigen::Vector3d(std::max(tmp_voxel_bb_1.x(), tmp_voxel_bb_2.x()),
                                                 std::max(tmp_voxel_bb_1.y(), tmp_voxel_bb_2.y()),
                                                 std::max(tmp_voxel_bb_1.z(), tmp_voxel_bb_2.z()));
    if (voxel_bb_1.x() > box_bb_2.x() || voxel_bb_2.x() < box_bb_1.x() || voxel_bb_1.y() > box_bb_2.y() ||
        voxel_bb_2.y() < box_bb_1.y())
    {
      overlap_vector[voxel_id] = 0;
    }
    else
    {
      double x_overlap = std::min(voxel_bb_2.x(), box_bb_2.x()) - std::max(voxel_bb_1.x(), box_bb_1.x());
      double y_overlap = std::min(voxel_bb_2.y(), box_bb_2.y()) - std::max(voxel_bb_1.y(), box_bb_1.y());
      double overlap_fraction = (x_overlap * y_overlap) / voxel_area;
      overlap_vector[voxel_id] = overlap_fraction;
    }

    ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "box_bb_1 \n" << box_bb_1);
    ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "box_bb_2 \n" << box_bb_2);
    ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "voxel_bb_1 \n" << voxel_bb_1);
    ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "voxel_bb_2 \n" << voxel_bb_2);
    ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "overlap_vector[voxel_id] \n" << overlap_vector[voxel_id]);

    if (visual_tools)
    {
      Eigen::Isometry3d vbb1 = Eigen::Translation3d(voxel_bb_1) * Eigen::Quaterniond(1, 0, 0, 0);
      Eigen::Isometry3d vbb2 = Eigen::Translation3d(voxel_bb_2) * Eigen::Quaterniond(1, 0, 0, 0);

      visual_tools->publishAxisLabeled(vbb1, "vbb1", rviz_visual_tools::XSMALL);
      visual_tools->publishAxisLabeled(vbb2, "vbb2", rviz_visual_tools::XSMALL);
      if (overlap_vector[voxel_id] > 0.75)
      {
        visual_tools->publishCuboid(voxel_bb_1, voxel_bb_2, rviz_visual_tools::GREEN);
      }
      else if (overlap_vector[voxel_id] > 0.25)
      {
        visual_tools->publishCuboid(voxel_bb_1, voxel_bb_2, rviz_visual_tools::YELLOW);
      }
      else
      {
        visual_tools->publishCuboid(voxel_bb_1, voxel_bb_2, rviz_visual_tools::RED);
      }
    }
  }
  if (visual_tools)
  {
    visual_tools->trigger();
    ros::Duration(0.1).sleep();
    // visual_tools->prompt("next");
  }

  return overlap_vector;
}

}  // end namespace moveit_grasps
