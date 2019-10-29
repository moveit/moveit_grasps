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

#include <moveit_grasps/two_finger_grasp_scorer.h>

namespace moveit_grasps
{
double TwoFingerGraspScorer::scoreGraspWidth(const TwoFingerGraspDataPtr grasp_data, double percent_open)
{
  ROS_DEBUG_STREAM_NAMED("grasp_scorer.graspWidth", "raw score = " << percent_open);
  return pow(percent_open, 2);
}

double TwoFingerGraspScorer::scoreDistanceToPalm(const Eigen::Isometry3d& grasp_pose_tcp,
                                                 const TwoFingerGraspDataPtr grasp_data,
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

}  // end namespace moveit_grasps
