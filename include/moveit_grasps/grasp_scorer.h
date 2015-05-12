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
 * Authors : Andy Mcevoy
 * Desc    : Functions for scoreing generated grasps
 */

#ifndef MOVEIT_GRASPS_GRASP_SCORER_
#define MOVEIT_GRASPS_GRASP_SCORER_

#include <Eigen/Core>

namespace moveit_grasps
{

class GraspScorer
{
public:

  /**
   * \brief Scores the grasp on how wide the fingers are on approach, the more open the better
   * \param grasp_pose - the pose of the end effector
   * \param grasp_data - pointer to grasp info
   * \param object_pose - the pose of the object being grasped
   * \return the unweighted score: 1.0 -> gripper is wide open, 0.0 -> gripper is fully closed.
   */
  static double scoreGraspWidth(const Eigen::Affine3d& grasp_pose, 
                                const GraspDataPtr grasp_data, 
                                const Eigen::Affine3d& object_pose);

  /**
   * \brief Scores each axis of the grasp based on its angle to the desired pose axis.
   * \param grasp_pose - the pose of the end effector
   * \param grasp_data - pointer to grasp info
   * \param object_pose - the pose of the object being grasped
   * \return the unweighted scores: 1.0 -> 0 degrees between grasp axis and desired axis, 0.0 -> 180 degree
   */
  static Eigen::Vector3d scoreRotationsFromDesired(const Eigen::Affine3d& grasp_pose, 
                                                   const GraspDataPtr grasp_data, 
                                                   const Eigen::Affine3d& object_pose);

  /**
   * \brief Score the grasp based on how far the object is from the palm of the hand
   * \param grasp_pose - the pose of the end effector
   * \param grasp_data - pointer to grasp info
   * \param object_pose - the pose of the object being grasped
   * \return the unweighted score: 1.0 -> object is touching the palm, 0.0 -> object is at max distanct
   */
  static double scoreDistanceToPalm(const Eigen::Affine3d& grasp_pose, 
                                    const GraspDataPtr grasp_data, 
                                    const Eigen::Affine3d& object_pose);

};

} // end namespace moveit_grasps

#endif
