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
 * Authors : Andy McEvoy
 * Desc    : Functions for scoring generated grasps
 */

#ifndef MOVEIT_GRASPS_GRASP_SCORER_
#define MOVEIT_GRASPS_GRASP_SCORER_

#include <cmath>

#include <ros/ros.h>

#include <moveit_grasps/grasp_data.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace moveit_grasps
{
class GraspScorer
{
public:
  /**
   * \brief Scores the grasp on how wide the fingers are on approach, the more open the better
   * \param grasp_data - pointer to grasp info
   * \param percent_open - amount the gripper is open
   *                       0.0 -> gripper is open to the object width + minimum padding
   *                       1.0 -> gripper is in full open position
   * \return the unweighted score:
   *         1.0 -> gripper is wide open,
   *         0.0 -> gripper is at minimum position.
   */
  static double scoreGraspWidth(const GraspDataPtr grasp_data, double percent_open);

  /**
   * \brief Scores each axis of the grasp based on its angle to the desired pose axis.
   * \param grasp_pose - the pose of the end effector
   * \param ideal_pose - the ideal grasp pose (ex: straight into the bin)
   * \return the unweighted scores:
   *         1.0 -> 0 degrees between grasp axis and desired axis,
   *         0.0 -> 180 degrees
   */
  static Eigen::Vector3d scoreRotationsFromDesired(const Eigen::Affine3d& grasp_pose,
                                                   const Eigen::Affine3d& ideal_pose);

  /**
   * \brief Score the grasp based on how far the object is from the palm of the hand
   * \param grasp_pose - the pose of the end effector
   * \param grasp_data - pointer to grasp info
   * \param object_pose - the pose of the object being grasped
   * \param max_grasp_distance - the maximum acceptable distance from palm
   * \return the unweighted score:
   *         1.0 -> object pose and grasp pose have same translation values
   *         0.0 -> object is at max distanct
   *       < 0.0 -> object is beyond the max_grasp_distance
   */
  // DEV NOTE: when this function is called we've lost the references to the acutal size of the object.
  // max_distance should be the length of the fingers minus some minimum amount that the fingers need to grip an object
  // since we don't know the distance from the centoid of the object to the edge of the object, this is set as an
  // arbitrary number given our target object set (i.e. I based it off of the cheese it box)
  static double scoreDistanceToPalm(const Eigen::Affine3d& grasp_pose, const GraspDataPtr grasp_data,
                                    const Eigen::Affine3d& object_pose, const double& min_grasp_distance,
                                    const double& max_grasp_distance);

  /**
   * \brief Score the grasp based on the translation values of the grasp pose
   * \param grasp_pose - the pose of the end effector
   * \param min_translations - the minimum translation values for all grasp poses
   * \param max_translations - the maximum translation values for all grasp poses
   * \return the unweighted scores:
   *         0.0 -> pose is at the minimum translation in that axis
   *         1.0 -> pose is at the maximum translation in that axis
   */
  static Eigen::Vector3d scoreGraspTranslation(const Eigen::Affine3d& grasp_pose,
                                               const Eigen::Vector3d& min_translations,
                                               const Eigen::Vector3d& max_translations);
};

}  // end namespace moveit_grasps

#endif
