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

/* Authors: Dave Coleman, Bence Magyar
   Description: Data class used by the grasp generator.
*/

#ifndef MOVEIT_GRASPS__TWO_FINGER_GRASP_DATA_H_
#define MOVEIT_GRASPS__TWO_FINGER_GRASP_DATA_H_

// moveit grasps
#include <moveit_grasps/grasp_data.h>

namespace moveit_grasps
{
MOVEIT_CLASS_FORWARD(TwoFingerGraspData);

class TwoFingerGraspData : public GraspData
{
public:
  /**
   * \brief Loads grasp data from a yaml file (load from roslaunch)
   * \param node handle - allows for namespacing
   * \param end effector name - which side of a two handed robot to load data for. should correspond to SRDF EE names
   */
  TwoFingerGraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                     moveit::core::RobotModelConstPtr robot_model);

  /**
   * \brief Helper function for constructor
   * \return true on success
   */
  bool loadGraspData(const ros::NodeHandle& nh, const std::string& end_effector) override;

  /**
   * \brief Set the width between fingers as a percentage of object size and max finger width
   * \return true on success
   */
  bool setGraspWidth(const double& percent_open, const double& min_finger_width,
                     trajectory_msgs::JointTrajectory& grasp_posture);

  /**
   * \brief Convert width between fingers to joint positions
   * \return true on success
   */
  bool fingerWidthToGraspPosture(const double& distance_btw_fingers, trajectory_msgs::JointTrajectory& grasp_posture);

  /**
   * \brief Convert joint positions to full grasp posture
   * \return true on success
   */
  bool jointPositionsToGraspPosture(std::vector<double> joint_positions,
                                    trajectory_msgs::JointTrajectory& grasp_posture);

  /**
   * \brief Debug data to console
   */
  void print() override;

public:
  /////////////////////////////////////
  // Finger gripper specific parameters
  /////////////////////////////////////
  // For calculating the ratio between the distance between fingers and the joint values
  double max_grasp_width_;
  double max_finger_width_;
  double min_finger_width_;
  double gripper_finger_width_;  // parameter used to ensure generated grasps will overlap object
};

}  // namespace

#endif
