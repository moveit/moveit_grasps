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
  return pow(percent_open,4);
}

double GraspScorer::scoreDistanceToPalm(const Eigen::Affine3d& grasp_pose, const GraspDataPtr grasp_data, 
                                  const Eigen::Affine3d& object_pose, const double& max_grasp_distance)
{
  // DEV NOTE: grasp_data is not used but should be. See *.h for explaination.

  double distance = ( grasp_pose.translation() - object_pose.translation() ).norm();
  double score = ( max_grasp_distance - std::abs(distance) ) / max_grasp_distance;

  return pow(score,4);
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
  angle = acos( grasp_pose_axis.dot(ideal_pose_axis) );
  scores[0] = (M_PI - angle) / M_PI;

  // get angle between y-axes
  grasp_pose_axis = grasp_pose.rotation() * Eigen::Vector3d::UnitY();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitY();
  angle = acos( grasp_pose_axis.dot(ideal_pose_axis) );
  scores[1] = (M_PI - angle) / M_PI;

  // get angle between z-axes
  grasp_pose_axis = grasp_pose.rotation() * Eigen::Vector3d::UnitZ();
  ideal_pose_axis = ideal_pose.rotation() * Eigen::Vector3d::UnitZ();
  angle = acos( grasp_pose_axis.dot(ideal_pose_axis) );
  scores[2] = (M_PI - angle) / M_PI;

  return scores;
}

} // end namespace moveit_grasps
