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

/* Author: Dave Coleman <dave@picknik.ai>
   Desc:   Filters grasps based on kinematic feasibility and collision
*/

#ifndef MOVEIT_GRASPS__TWO_FINGER_GRASP_FILTER_
#define MOVEIT_GRASPS__TWO_FINGER_GRASP_FILTER_

// Parent class
#include <moveit_grasps/grasp_filter.h>

// Grasping
#include <moveit_grasps/two_finger_grasp_generator.h>

namespace moveit_grasps
{
class TwoFingerGraspFilter : public GraspFilter
{
public:
  // Constructor
  TwoFingerGraspFilter(const robot_state::RobotStatePtr& robot_state,
                       const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools);

  /**
   * \brief Thread for checking part of the possible grasps list
   */
  bool processCandidateGrasp(const IkThreadStructPtr& ik_thread_struct) override;

  /**
   * \brief Check if ik solution is in collision with fingers closed
   * \return true on success
   */
  bool checkFingersClosedIK(std::vector<double>& ik_solution, const IkThreadStructPtr& ik_thread_struct,
                            GraspCandidatePtr& grasp_candidate,
                            const moveit::core::GroupStateValidityCallbackFn& constraint_fn) const;

protected:
  // Name for logging
  const std::string name_;

};  // end of class

typedef std::shared_ptr<TwoFingerGraspFilter> TwoFingerGraspFilterPtr;
typedef std::shared_ptr<const TwoFingerGraspFilter> TwoFingerGraspFilterConstPtr;

}  // namespace moveit_grasps

#endif
