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
   Desc:   Filters grasps based on kinematic feasibility and collision
*/

#ifndef MOVEIT_GRASPS__GRASP_FILTER_
#define MOVEIT_GRASPS__GRASP_FILTER_

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/Grasp.h>

// Grasping
#include <moveit_grasps/grasp_generator.h>

// Rviz
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// C++
#include <boost/thread.hpp>
#include <math.h>
#define _USE_MATH_DEFINES

namespace moveit_grasps
{

/**
 * \brief Contains collected data for each potential grasp after it has been verified / filtered
 */
struct GraspCandidate
{
  GraspCandidate(moveit_msgs::Grasp grasp)
    : grasp_(grasp)
    , valid_(false)
    , grasp_filtered_by_ik_(false)
    , grasp_filtered_by_collision_(false)
    , pregrasp_filtered_by_ik_(false)
    , pregrasp_filtered_by_collision_(false)
  {}

  moveit_msgs::Grasp grasp_;
  std::vector<double> grasp_ik_solution_;
  std::vector<double> pregrasp_ik_solution_;

  bool valid_;
  bool grasp_filtered_by_ik_;
  bool grasp_filtered_by_collision_;
  bool pregrasp_filtered_by_ik_;
  bool pregrasp_filtered_by_collision_;
};
typedef boost::shared_ptr<GraspCandidate> GraspCandidatePtr;

/**
 * \brief Struct for passing parameters to threads, for cleaner code
 */
struct IkThreadStruct
{
  IkThreadStruct(
                 std::vector<GraspCandidatePtr> &grasp_candidates, // the input
                 planning_scene::PlanningScenePtr planning_scene,
                 Eigen::Affine3d &link_transform,
                 int grasps_id_start,
                 int grasps_id_end,
                 kinematics::KinematicsBaseConstPtr kin_solver,
                 const robot_model::JointModelGroup* ee_jmg,
                 const robot_model::JointModelGroup* arm_jmg,
                 double timeout,
                 bool filter_pregrasp,
                 bool verbose,
                 bool collision_verbose,
                 int thread_id)
  : grasp_candidates_(grasp_candidates),
    planning_scene_(planning_scene),
    link_transform_(link_transform),
    grasps_id_start_(grasps_id_start),
    grasps_id_end_(grasps_id_end),
    kin_solver_(kin_solver),
    ee_jmg_(ee_jmg),
    arm_jmg_(arm_jmg),
    timeout_(timeout),
    filter_pregrasp_(filter_pregrasp),
    verbose_(verbose),
    collision_verbose_(collision_verbose),
    thread_id_(thread_id)
  {
  }
  std::vector<GraspCandidatePtr> &grasp_candidates_;
  planning_scene::PlanningScenePtr planning_scene_;
  Eigen::Affine3d link_transform_;
  int grasps_id_start_;
  int grasps_id_end_;
  kinematics::KinematicsBaseConstPtr kin_solver_;
  const robot_model::JointModelGroup* ee_jmg_;
  const robot_model::JointModelGroup* arm_jmg_;
  double timeout_;
  bool filter_pregrasp_;
  bool verbose_;
  bool collision_verbose_;
  int thread_id_;
};

// Class
class GraspFilter
{
public:

  // Constructor
  GraspFilter( robot_state::RobotStatePtr robot_state,
               moveit_visual_tools::MoveItVisualToolsPtr& visual_tools );

  /**
   * \brief Convert the ROS message vector into our custom struct for candidate grasps
   * \param vector of grasps generated from a grasp generator
   * \return vector of grasps in this package's proper format
   */
  std::vector<GraspCandidatePtr> convertToGraspCandidatePtrs(const std::vector<moveit_msgs::Grasp>& grasp_candidates);

  /**
   * \brief Return grasps that are kinematically feasible
   * \param grasp_candidates - all possible grasps that this will test. this vector is returned modified
   * \param arm_jmg - the arm to solve the IK problem on
   * \param filter_pregrasp -whether to also check ik feasibility for the pregrasp position
   * \param verbose_if_failed - show debug markers and logging if no grasps where found the first time
   * \return number of grasps remaining
   */
  std::size_t filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                           planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                           const robot_model::JointModelGroup* arm_jmg,
                           bool filter_pregrasp = false,
                           bool verbose = false,
                           bool verbose_if_failed = true,
                           bool collision_verbose = false);

  /**
   * \brief Helper for filterGrasps
   * \return number of grasps remaining
   */
  std::size_t filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                 planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor,
                                 const robot_model::JointModelGroup* ee_jmg,
                                 const robot_model::JointModelGroup* arm_jmg,
                                 bool filter_pregrasp,
                                 bool verbose, bool collision_verbose);

  /**
   * \brief Thread for checking part of the possible grasps list
   */
  void filterGraspsThread(IkThreadStruct ik_thread_struct);

  /**
   * \brief Helper for the thread function to find IK solutions
   * \return true on success
   */
  bool findIKSolution(geometry_msgs::PoseStamped& ik_pose, std::vector<double>& ik_solution,
                      std::vector<double>& ik_seed_state, moveit_msgs::MoveItErrorCodes& error_code,
                      IkThreadStruct& ik_thread_struct);

  /**
   * \brief Helper for the thread function to check the arm for collision with the planning scene
   * \return true on success
   */
  bool checkInCollision(std::vector<double>& ik_solution,
                        IkThreadStruct& ik_thread_struct,
                        bool verbose);

  /**
   * \brief Of an array of grasps, choose just one for use
   * \return true on success
   */
  bool chooseBestGrasp( std::vector<GraspCandidatePtr>& grasp_candidates,
                        GraspCandidatePtr& chosen );

  /**
   * \brief Show grasps after being filtered
   * \return true on success
   */
  bool visualizeGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates,
                       const moveit::core::JointModelGroup *arm_jmg);

  /**
   * \brief Show IK solutions of entire arm
   * \return true on success
   */
  bool visualizeIKSolutionsOLD(const std::vector<GraspCandidatePtr>& grasp_candidates,
                               const moveit::core::JointModelGroup* arm_jmg, double animation_speed);

  /**
   * \brief Show IK solutions of entire arm
   * \return true on success
   */
  bool visualizeIKSolutions(const std::vector<GraspCandidatePtr>& grasp_candidates,
                            const moveit::core::JointModelGroup* arm_jmg, double animation_speed);

private:

  // Allow a writeable robot state
  robot_state::RobotStatePtr robot_state_;

  // Threaded kinematic solvers
  std::map<std::string, std::vector<kinematics::KinematicsBaseConstPtr> > kin_solvers_;

  // Class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Number of degrees of freedom for the IK solver to find
  std::size_t num_variables_;

  // Time to allow IK solver to run
  double solver_timeout_;

  // Visualization levels
  bool show_filtered_grasps_;
  bool show_filtered_arm_solutions_;
  bool collision_verbose_;

  // Shared node handle
  ros::NodeHandle nh_;

}; // end of class

typedef boost::shared_ptr<GraspFilter> GraspFilterPtr;
typedef boost::shared_ptr<const GraspFilter> GraspFilterConstPtr;

} // namespace

#endif
