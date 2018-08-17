/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, University of Colorado, Boulder
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
 *   * Neither the name of the Univ of CO, Boulder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
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
 *********************************************************************/

/* Author: Dave Coleman <dave@dav.ee>
   Desc:   Callback for checking if a state is in collision
*/

#ifndef MOVEIT_GRASPS__STATE_VALIDITY_CALLBACK
#define MOVEIT_GRASPS__STATE_VALIDITY_CALLBACK

namespace
{
bool isGraspStateValid(const planning_scene::PlanningScene *planning_scene, bool verbose, double verbose_speed,
                       moveit_visual_tools::MoveItVisualToolsPtr visual_tools, robot_state::RobotState *robot_state,
                       const robot_state::JointModelGroup *group, const double *ik_solution)
{
  robot_state->setJointGroupPositions(group, ik_solution);
  robot_state->update();

  if (!planning_scene)
  {
    ROS_ERROR_STREAM_NAMED("manipulation", "No planning scene provided");
    return false;
  }
  if (!planning_scene->isStateColliding(*robot_state, group->getName()))
    return true;  // not in collision

  // Display more info about the collision
  if (verbose)
  {
    visual_tools->publishRobotState(*robot_state, rviz_visual_tools::RED);
    planning_scene->isStateColliding(*robot_state, group->getName(), true);
    visual_tools->publishContactPoints(*robot_state, planning_scene);
    ros::Duration(verbose_speed).sleep();
  }
  return false;
}

}  // namespace

#endif
