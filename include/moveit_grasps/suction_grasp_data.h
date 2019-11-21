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

/* Authors: Michael Lautman
   Description: Data class used by the grasp generator.
*/

#ifndef MOVEIT_GRASPS__SUCTION_GRASP_DATA_H_
#define MOVEIT_GRASPS__SUCTION_GRASP_DATA_H_

// moveit grasps
#include <moveit_grasps/grasp_data.h>
#include <moveit_grasps/suction_voxel_matrix.h>

namespace moveit_grasps
{
MOVEIT_CLASS_FORWARD(SuctionGraspData);

struct SuctionGraspData : public GraspData
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /**
   * \brief Creates a suction grasp data object
   * \param node_handle - allows for namespacing
   * \param end_effector name - which side of a two handed robot to load data for. should correspond to SRDF EE names
   * \param robot_model - The robot model
   */
  SuctionGraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                   const moveit::core::RobotModelConstPtr& robot_model);

  /**
   * \brief Helper function that loads grasp data from a yaml file (load from roslaunch)
   * \param nh - node handle allows for namespacing
   * \param end_effector - The end effector joint group name
   * \return true on success
   */
  bool loadGraspData(const ros::NodeHandle& nh, const std::string& end_effector) override;

  /**
   * \brief Debug data to console
   */
  void print() override;

public:
  //////////////////////////////////////
  // Suction gripper specific parameters
  //////////////////////////////////////
  std::shared_ptr<SuctionVoxelMatrix> suction_voxel_matrix_;
};

}  // namespace moveit_grasps

#endif
