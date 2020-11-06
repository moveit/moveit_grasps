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
   * \brief Creates a two finger grasp data object
   * \param node_handle - allows for namespacing
   * \param end_effector name - which side of a two handed robot to load data for. should correspond to SRDF EE names
   * \param robot_model - The robot model
   */
  TwoFingerGraspData(const ros::NodeHandle& nh, const std::string& end_effector,
                     const moveit::core::RobotModelConstPtr& robot_model);

  /**
   * \brief Helper function for constructor, loads grasp data from a yaml file (load from roslaunch)
   * \param nh - node handle allows for namespacing
   * \param end_effector - The end effector joint group name
   * \return true on success
   */
  bool loadGraspData(const ros::NodeHandle& nh, const std::string& end_effector) override;

  /**
   * \brief Set the width between fingers as a percentage of object size and max finger width
   * \param fraction_open - [0,1] the 0->closed 1->open
   * \return true on success
   */
  bool setGraspWidth(double fraction_open, double min_finger_width, trajectory_msgs::JointTrajectory& grasp_posture);

  /**
   * \brief Convert width between fingers to joint positions
   * \param distance_btw_fingers - (meters) The target distance between the fingers
   * \param grasp_posture - output. A joint trajectory with the values for the end effector filled in.
   * Note: we interpolate to get the joint value assuming a linear relationship between min and max
   * finger distance and min and max finger joint value. This could likely be improved on a per-robot basis.
   * \param end_effector - The end effector joint group name
   * \return true on success
   */
  bool fingerWidthToGraspPosture(double distance_btw_fingers, trajectory_msgs::JointTrajectory& grasp_posture);

  /**
   * \brief Convert joint positions to full grasp posture
   * \param joint_positions - the full joint state as a vector of doubles
   * \param grasp_posture - output. The full grasp posture
   * \return true on success
   */
  bool jointPositionsToGraspPosture(const std::vector<double>& joint_positions,
                                    trajectory_msgs::JointTrajectory& grasp_posture);

  /**
   * \brief Debug data to console
   */
  void print() override;

public:
  /////////////////////////////////////
  // Finger gripper specific parameters
  /////////////////////////////////////
  // Maximum allowed finger width for a grasp.
  // This value should be considerably smaller than max_finger_width
  // to allow padded collision checks
  double max_grasp_width_;
  // Maximum / Minimum distance between fingers
  // For calculating the ratio between the distance between fingers and the joint values
  double max_finger_width_;
  double min_finger_width_;
  // Parameter used to ensure generated grasps will overlap object
  double gripper_finger_width_;
};

}  // namespace moveit_grasps

#endif
