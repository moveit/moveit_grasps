/*
 * Software License Agreement (Modified BSD License)
 *
 *  Copyright (c) 2014, University of Colorado, Boulder, PAL Robotics, S.L.
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
 *   * Neither the name of Univ of CO, Boulder, PAL Robotics, S.L.
 *     nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific
 *     prior written permission.
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
 */

/* Authors: Mike Lautman
   Description: Data class used by grasp data.
*/

#ifndef MOVEIT_GRASPS__SUCTION_VOXEL_MATRIX_H_
#define MOVEIT_GRASPS__SUCTION_VOXEL_MATRIX_H_

// ROS
#include <ros/ros.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace moveit_grasps
{
struct SuctionVoxel
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /* \brief
    @param center_point - position of voxel center point in tcp frame
    @param x_width - width of voxel along x dim in tcp frame
    @param y_width - width of voxel along y dim in tcp frame
   */
  SuctionVoxel(const Eigen::Vector3d& center_point, double x_width, double y_width)
    : center_point_(center_point), x_width_(x_width), y_width_(y_width)
  {
    top_left_ = center_point + Eigen::Vector3d(-x_width / 2.0, y_width / 2.0, 0);
    top_right_ = center_point + Eigen::Vector3d(x_width / 2.0, y_width / 2.0, 0);
    bottom_left_ = center_point + Eigen::Vector3d(-x_width / 2.0, -y_width / 2.0, 0);
    bottom_right_ = center_point + Eigen::Vector3d(x_width / 2.0, -y_width / 2.0, 0);
  }

  // Voxel center point in tcp frame
  Eigen::Vector3d center_point_;
  double x_width_;
  double y_width_;
  // Voxel corners in tcp frame
  Eigen::Vector3d top_left_;
  Eigen::Vector3d top_right_;
  Eigen::Vector3d bottom_left_;
  Eigen::Vector3d bottom_right_;
};

class SuctionVoxelMatrix
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SuctionVoxelMatrix(double suction_rows_count, double suction_cols_count, double total_suction_range_y,
                     double total_suction_range_x)
    : suction_rows_count_(suction_rows_count)
    , suction_cols_count_(suction_cols_count)
    , active_suction_range_x_(total_suction_range_x)
    , active_suction_range_y_(total_suction_range_y)
  {
    voxel_x_width_ = active_suction_range_x_ / suction_cols_count_;
    voxel_y_width_ = active_suction_range_y_ / suction_rows_count_;
    suction_voxels_.resize(suction_rows_count_);
    // We store the voxels starting bottom left and moving right then up
    for (std::size_t voxel_y = 0; voxel_y < suction_rows_count_; ++voxel_y)
    {
      suction_voxels_[voxel_y].resize(suction_cols_count_);
      for (std::size_t voxel_x = 0; voxel_x < suction_cols_count_; ++voxel_x)
      {
        suction_voxels_[voxel_y][voxel_x] = std::make_shared<SuctionVoxel>(
            Eigen::Vector3d(-active_suction_range_x_ / 2.0 + voxel_x_width_ * (voxel_x + 0.5),
                            -active_suction_range_y_ / 2.0 + voxel_y_width_ * (voxel_y + 0.5), 0),
            voxel_x_width_, voxel_y_width_);
      }
    }
  }

  // \brief - get the voxel at the index location [row, col] with [0, 0] being the bottom left
  bool getSuctionVoxel(std::size_t row, std::size_t col, std::shared_ptr<SuctionVoxel>& voxel)
  {
    if (row >= suction_rows_count_)
    {
      ROS_DEBUG_STREAM_NAMED("suction_voxel_matrix", "Invalid row " << row << "/" << suction_rows_count_ - 1);
      return false;
    }

    if (col >= suction_cols_count_)
    {
      ROS_DEBUG_STREAM_NAMED("suction_voxel_matrix", "Invalid col " << col << "/" << suction_cols_count_ - 1);
      return false;
    }

    voxel = suction_voxels_[row][col];
    return true;
  }

  /** \brief Get the voxel at the index i where columns are the minor axis and rows are the major axes.
   *  @param index - the index of the suction voxel where index / #cols is the row and index % #cols is the col
   *                 index 0 is bottom left
   */
  bool getSuctionVoxel(std::size_t index, std::shared_ptr<SuctionVoxel>& voxel)
  {
    return getSuctionVoxel(index / suction_cols_count_, index % suction_cols_count_, voxel);
  }

  std::size_t getNumRows()
  {
    return suction_rows_count_;
  }

  std::size_t getNumCols()
  {
    return suction_cols_count_;
  }

  std::size_t getNumVoxels()
  {
    return suction_cols_count_ * suction_rows_count_;
  }

  double getVoxelArea()
  {
    return voxel_x_width_ * voxel_y_width_;
  }

  double getVoxelWidthX()
  {
    return voxel_x_width_;
  }

  double getVoxelWidthY()
  {
    return voxel_y_width_;
  }

  double getActiveSuctionWidthX()
  {
    return active_suction_range_x_;
  }

  double getActiveSuctionWidthY()
  {
    return active_suction_range_y_;
  }

protected:
  std::size_t suction_rows_count_;
  std::size_t suction_cols_count_;
  double voxel_x_width_;
  double voxel_y_width_;
  double active_suction_range_x_;
  double active_suction_range_y_;
  std::vector<std::vector<std::shared_ptr<SuctionVoxel>>> suction_voxels_;
};

}  // namespace moveit_grasps

#endif
