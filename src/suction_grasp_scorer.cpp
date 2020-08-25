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

#include <moveit_grasps/suction_grasp_scorer.h>

namespace moveit_grasps
{
// is a within epsilon of b
bool isApprox(double a, double b, double epsilon = 1.0e-5)
{
  return std::abs(a - b) < epsilon;
}

double SuctionGraspScoreWeights::computeScore(const Eigen::Vector3d& orientation_scores,
                                              const Eigen::Vector3d& translation_scores, double overhang_score,
                                              bool verbose) const
{
  double total_score = GraspScoreWeights::computeScore(orientation_scores, translation_scores, false) *
                       GraspScoreWeights::getWeightTotal();
  total_score += overhang_score * overhang_score_weight_;

  total_score /= getWeightTotal();

  if (verbose)
  {
    static const std::string logger_name = "grasp_scorer.compute_score";
    // clang-format off
    ROS_DEBUG_STREAM_NAMED(logger_name, "Suction Grasp score: ");
    ROS_DEBUG_STREAM_NAMED(logger_name, "\torientation_score.x = " << orientation_scores[0] << "\tweight = "<< orientation_x_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\torientation_score.y = " << orientation_scores[1] << "\tweight = "<< orientation_y_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\torientation_score.z = " << orientation_scores[2] << "\tweight = "<< orientation_z_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttranslation_score.x = " << translation_scores[0] << "\tweight = "<< translation_x_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttranslation_score.y = " << translation_scores[1] << "\tweight = "<< translation_y_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttranslation_score.z = " << translation_scores[2] << "\tweight = "<< translation_z_score_weight_);
    ROS_DEBUG_STREAM_NAMED(logger_name, "\toverhang_score      = " << overhang_score        << "\tweight = "<< overhang_score_weight_);
    // Total
    ROS_DEBUG_STREAM_NAMED(logger_name, "\ttotal_score = " << total_score);
    // clang-format on
  }
  return total_score;
}

double SuctionGraspScoreWeights::getWeightTotal() const
{
  return GraspScoreWeights::getWeightTotal() + overhang_score_weight_;
}

double SuctionGraspScorer::scoreSuctionVoxelOverlap(const Eigen::Isometry3d& grasp_pose_tcp,
                                                    const SuctionGraspDataPtr& grasp_data,
                                                    const Eigen::Isometry3d& object_pose,
                                                    const Eigen::Vector3d& object_size,
                                                    std::vector<double>& overlap_vector,
                                                    moveit_visual_tools::MoveItVisualToolsPtr visual_tools)
{
  overlap_vector.resize(grasp_data->suction_voxel_matrix_->getNumVoxels());

  // These are in the object pose frame
  Eigen::Vector3d box_bb_max(object_size.x() / 2.0, object_size.y() / 2.0, 0);
  Eigen::Vector3d box_bb_min(-object_size.x() / 2.0, -object_size.y() / 2.0, 0);

  if (visual_tools)
  {
    visual_tools->deleteAllMarkers();
    visual_tools->trigger();
    visual_tools->publishAxisLabeled(grasp_pose_tcp, "tcp");
    Eigen::Isometry3d bbb1 = object_pose * Eigen::Translation3d(box_bb_max);
    Eigen::Isometry3d bbb2 = object_pose * Eigen::Translation3d(box_bb_min);
    visual_tools->publishAxisLabeled(bbb1, "bbb1", rviz_visual_tools::SMALL);
    visual_tools->publishAxisLabeled(bbb2, "bbb2", rviz_visual_tools::SMALL);
    visual_tools->trigger();
  }

  const double kVisualBuffer = 0.0001;
  const double kSlices = 10;
  double voxel_area = grasp_data->suction_voxel_matrix_->getVoxelArea();
  Eigen::Isometry3d grasp_pose_tcp_in_box_frame = object_pose.inverse() * grasp_pose_tcp;
  for (std::size_t voxel_id = 0; voxel_id < grasp_data->suction_voxel_matrix_->getNumVoxels(); ++voxel_id)
  {
    std::shared_ptr<SuctionVoxel> voxel;
    if (!grasp_data->suction_voxel_matrix_->getSuctionVoxel(voxel_id, voxel))
    {
      ROS_ERROR_STREAM_NAMED("grasp_scorer.voxels", "Invalid voxel id: " << voxel_id);
      overlap_vector.clear();
      overlap_vector.resize(grasp_data->suction_voxel_matrix_->getNumVoxels());
      return 0;
    }

    // These are also in the object pose frame
    Eigen::Vector3d tmp_voxel_bb_1 = grasp_pose_tcp_in_box_frame * voxel->bottom_left_;
    Eigen::Vector3d tmp_voxel_bb_2 = grasp_pose_tcp_in_box_frame * voxel->top_left_;
    Eigen::Vector3d tmp_voxel_bb_3 = grasp_pose_tcp_in_box_frame * voxel->top_right_;
    Eigen::Vector3d tmp_voxel_bb_4 = grasp_pose_tcp_in_box_frame * voxel->bottom_right_;

    std::vector<double> slope(4);
    std::vector<double> intercept(4);
    slope[0] = (tmp_voxel_bb_1.y() - tmp_voxel_bb_2.y()) / (tmp_voxel_bb_1.x() - tmp_voxel_bb_2.x());
    slope[1] = (tmp_voxel_bb_2.y() - tmp_voxel_bb_3.y()) / (tmp_voxel_bb_2.x() - tmp_voxel_bb_3.x());
    slope[2] = (tmp_voxel_bb_3.y() - tmp_voxel_bb_4.y()) / (tmp_voxel_bb_3.x() - tmp_voxel_bb_4.x());
    slope[3] = (tmp_voxel_bb_4.y() - tmp_voxel_bb_1.y()) / (tmp_voxel_bb_4.x() - tmp_voxel_bb_1.x());
    intercept[0] = tmp_voxel_bb_1.y() - slope[0] * tmp_voxel_bb_1.x();
    intercept[1] = tmp_voxel_bb_2.y() - slope[1] * tmp_voxel_bb_2.x();
    intercept[2] = tmp_voxel_bb_3.y() - slope[2] * tmp_voxel_bb_3.x();
    intercept[3] = tmp_voxel_bb_4.y() - slope[3] * tmp_voxel_bb_4.x();

    double max_y = std::max({ tmp_voxel_bb_1.y(), tmp_voxel_bb_2.y(), tmp_voxel_bb_3.y(), tmp_voxel_bb_4.y() });
    double min_y = std::min({ tmp_voxel_bb_1.y(), tmp_voxel_bb_2.y(), tmp_voxel_bb_3.y(), tmp_voxel_bb_4.y() });
    double y_inc = (max_y - min_y) / kSlices;
    std::vector<double> slice_overlap(kSlices);
    overlap_vector[voxel_id] = 0;
    for (std::size_t slice_ix = 0; slice_ix < kSlices - 0; ++slice_ix)
    {
      double y = min_y + y_inc * slice_ix;
      std::vector<double> x_intercept(4);
      // If the voxel is axis aligned with the bounding box, the slope intercept approach won't be valid
      if (isApprox(tmp_voxel_bb_1.x(), tmp_voxel_bb_2.x()) || isApprox(tmp_voxel_bb_2.x(), tmp_voxel_bb_3.x()))
      {
        x_intercept[1] = std::min(tmp_voxel_bb_1.x(), tmp_voxel_bb_3.x());
        x_intercept[2] = std::max(tmp_voxel_bb_1.x(), tmp_voxel_bb_3.x());
      }
      else
      {
        // We compute the x intercepts for each line of the box and take the middle two.
        for (std::size_t quadrant_ix = 0; quadrant_ix < 4; ++quadrant_ix)
          x_intercept[quadrant_ix] = (y - intercept[quadrant_ix]) / slope[quadrant_ix];
        std::sort(x_intercept.begin(), x_intercept.end());
      }

      Eigen::Vector3d voxel_slice_bb_min = Eigen::Vector3d(x_intercept[1], y, 0);
      Eigen::Vector3d voxel_slice_bb_max = Eigen::Vector3d(x_intercept[2], y + y_inc, 0);

      double overlap = 0;
      double y_overlap = 0;
      double x_overlap = 0;
      if (voxel_slice_bb_max.x() > box_bb_min.x() && voxel_slice_bb_min.x() < box_bb_max.x() &&
          voxel_slice_bb_max.y() > box_bb_min.y() && voxel_slice_bb_min.y() < box_bb_max.y() &&
          !isApprox(voxel_slice_bb_max.x(), voxel_slice_bb_min.x()))
      {
        y_overlap = std::min(box_bb_max.y(), y + y_inc) - std::max(box_bb_min.y(), y);
        x_overlap = std::min(voxel_slice_bb_max.x(), box_bb_max.x()) - std::max(voxel_slice_bb_min.x(), box_bb_min.x());
        overlap = x_overlap * y_overlap;
        overlap_vector[voxel_id] += overlap / voxel_area;
      }

      double voxel_slice_area =
          (voxel_slice_bb_max.x() - voxel_slice_bb_min.x()) * (voxel_slice_bb_max.y() - voxel_slice_bb_min.y());

      if (visual_tools)
      {
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "x_intercepts: " << x_intercept[0] << ",\t" << x_intercept[1]
                                                                       << ",\t" << x_intercept[2] << ",\t"
                                                                       << x_intercept[3]);

        // clang-format off
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "box_bb_max           " << box_bb_max.x() << ",\t" << box_bb_max.y() << ",\t" << box_bb_max.z());
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "box_bb_min           " << box_bb_min.x() << ",\t" << box_bb_min.y() << ",\t" << box_bb_min.z());
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "voxel_slice_bb_min   " << voxel_slice_bb_min.x() << ",\t" << voxel_slice_bb_min.y() << ",\t" << voxel_slice_bb_min.z());
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "voxel_slice_bb_max   " << voxel_slice_bb_max.x() << ",\t" << voxel_slice_bb_max.y() << ",\t" << voxel_slice_bb_max.z());
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "y inc                " << y_inc);
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "x overlap            " << x_overlap);
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "y overlap            " << y_overlap);
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "overlap              " << overlap);
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "voxel_slice_area     " << voxel_slice_area);
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "overlap / slice_area " << overlap / voxel_slice_area);
        ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels", "overlap_vector[" << voxel_id << "]     = " << overlap_vector[voxel_id]);
        // clang-format on
        Eigen::Vector3d voxel_slice_center_point_pos =
            Eigen::Vector3d((x_intercept[1] + x_intercept[2]) / 2, y + y_inc / 2, 0);
        Eigen::Isometry3d voxel_slice_center_point = object_pose * Eigen::Translation3d(voxel_slice_center_point_pos);
        Eigen::Isometry3d vsbb1 = object_pose * Eigen::Translation3d(voxel_slice_bb_min);
        Eigen::Isometry3d vsbb2 = object_pose * Eigen::Translation3d(voxel_slice_bb_max);

        visual_tools->publishAxisLabeled(vsbb1, "vsbb1", rviz_visual_tools::XXXSMALL);
        visual_tools->publishAxisLabeled(vsbb2, "vsbb2", rviz_visual_tools::XXXSMALL);

        double mesh_x_wid = x_intercept[2] - x_intercept[1] - kVisualBuffer;
        double mesh_y_wid = y_inc - kVisualBuffer;

        if (overlap / voxel_slice_area > 0.75)
        {
          visual_tools->publishWireframeCuboid(voxel_slice_center_point, mesh_x_wid, mesh_y_wid, 0.0001,
                                               rviz_visual_tools::GREEN);
        }
        else if (overlap / voxel_slice_area > 0.25)
        {
          visual_tools->publishWireframeCuboid(voxel_slice_center_point, mesh_x_wid, mesh_y_wid, 0.0001,
                                               rviz_visual_tools::YELLOW);
        }
        else
        {
          visual_tools->publishWireframeCuboid(voxel_slice_center_point, mesh_x_wid, mesh_y_wid, 0.0001,
                                               rviz_visual_tools::RED);
        }
      }
    }
    // normalize
    ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels.score",
                           "overlap_vector[" << voxel_id << "]     = " << overlap_vector[voxel_id]);

    if (visual_tools)
    {
      Eigen::Isometry3d voxel_center_point = grasp_pose_tcp * Eigen::Translation3d(voxel->center_point_);
      if (overlap_vector[voxel_id] > 0.75)
        visual_tools->publishWireframeCuboid(voxel_center_point, voxel->x_width_ - kVisualBuffer,
                                             voxel->y_width_ - kVisualBuffer, 0.001, rviz_visual_tools::GREEN);
      else if (overlap_vector[voxel_id] > 0.25)
        visual_tools->publishWireframeCuboid(voxel_center_point, voxel->x_width_ - kVisualBuffer,
                                             voxel->y_width_ - kVisualBuffer, 0.001, rviz_visual_tools::YELLOW);
      else
        visual_tools->publishWireframeCuboid(voxel_center_point, voxel->x_width_ - kVisualBuffer,
                                             voxel->y_width_ - kVisualBuffer, 0.001, rviz_visual_tools::RED);
      visual_tools->trigger();
    }
  }

  double overhang_score = 0;
  for (double voxel_overlap : overlap_vector)
    overhang_score += voxel_overlap * voxel_overlap;

  if (visual_tools)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_scorer.voxels.score", "overhang_score = " << overhang_score);
    visual_tools->trigger();
    ros::Duration(0.01).sleep();
    visual_tools->prompt("'next' to continue");
  }

  return overhang_score;
}

}  // namespace moveit_grasps
