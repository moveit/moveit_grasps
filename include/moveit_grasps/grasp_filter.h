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

#ifndef MOVEIT_GRASPS__GRASP_FILTER_
#define MOVEIT_GRASPS__GRASP_FILTER_

// ROS
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

// Grasping
#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_candidate.h>

// Rviz
#include <moveit_visual_tools/moveit_visual_tools.h>

// MoveIt
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/Grasp.h>
#include <moveit/kinematics_plugin_loader/kinematics_plugin_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

// C++
#include <boost/thread.hpp>
#include <cmath>

namespace moveit_grasps
{
enum GraspParallelPlane
{
  XY,
  XZ,
  YZ
};

/**
 * \brief Contains information to filter grasps by a cutting plane
 */
struct CuttingPlane
{
  Eigen::Isometry3d pose_;
  GraspParallelPlane plane_;
  int direction_;

  CuttingPlane(Eigen::Isometry3d pose, GraspParallelPlane plane, int direction)
    : pose_(pose), plane_(plane), direction_(direction)
  {
  }
};
typedef std::shared_ptr<CuttingPlane> CuttingPlanePtr;

/**
 * \brief Contains information to filter grasps by orientation
 */
struct DesiredGraspOrientation
{
  Eigen::Isometry3d pose_;
  double max_angle_offset_;

  DesiredGraspOrientation(Eigen::Isometry3d pose, double max_angle_offset)
    : pose_(pose), max_angle_offset_(max_angle_offset)
  {
  }
};
typedef std::shared_ptr<DesiredGraspOrientation> DesiredGraspOrientationPtr;

/**
 * \brief Struct for passing parameters to threads, for cleaner code
 */
struct IkThreadStruct
{
  IkThreadStruct(std::vector<GraspCandidatePtr>& grasp_candidates,  // the input
                 const planning_scene::PlanningScenePtr& planning_scene, Eigen::Isometry3d& link_transform,
                 std::size_t grasp_id, kinematics::KinematicsBaseConstPtr kin_solver,
                 const robot_state::RobotStatePtr& robot_state, double timeout, bool filter_pregrasp, bool visual_debug,
                 std::size_t thread_id, const std::string& grasp_target_object_id)
    : grasp_candidates_(grasp_candidates)
    , planning_scene_(planning_scene::PlanningScene::clone(planning_scene))
    , link_transform_(link_transform)
    , grasp_id(grasp_id)
    , kin_solver_(kin_solver)
    , robot_state_(std::make_shared<robot_state::RobotState>(*robot_state))
    , timeout_(timeout)
    , filter_pregrasp_(filter_pregrasp)
    , visual_debug_(visual_debug)
    , thread_id_(thread_id)
    , grasp_target_object_id_(grasp_target_object_id)
  {
  }

  std::vector<GraspCandidatePtr>& grasp_candidates_;
  planning_scene::PlanningScenePtr planning_scene_;
  Eigen::Isometry3d link_transform_;
  std::size_t grasp_id;
  kinematics::KinematicsBaseConstPtr kin_solver_;
  robot_state::RobotStatePtr robot_state_;
  double timeout_;
  bool filter_pregrasp_;
  bool visual_debug_;
  std::size_t thread_id_;
  // The name of the target grasp object to be used in the planning scene.
  // NOTE: The ik filtering checks if the user has already added the grasp object
  // to the planning scene. If it hasn't then GraspFilter will not modify the planning scene allowed collision matrix
  std::string grasp_target_object_id_;

  // Used within processing function
  geometry_msgs::PoseStamped ik_pose_;  // Set from grasp candidate
  std::vector<double> ik_seed_state_;
};
typedef std::shared_ptr<IkThreadStruct> IkThreadStructPtr;

class GraspFilter
{
public:
  // Constructor
  GraspFilter(const robot_state::RobotStatePtr& robot_state,
              const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools);

  /**
   * \brief Return grasps that are kinematically feasible
   * \param grasp_candidates - all possible grasps that this will test. this vector is returned modified
   * \param arm_jmg - the arm to solve the IK problem on
   * \param filter_pregrasp -whether to also check ik feasibility for the pregrasp position
   * \param target_object_id - The name of the target grasp object in the planning scene if it exists
   * \return true on successful filter
   */
  virtual bool filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                            const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                            const robot_model::JointModelGroup* arm_jmg, const moveit::core::RobotStatePtr& seed_state,
                            bool filter_pregrasp = false, const std::string& target_object_id = "");

  virtual bool filterGrasps(std::vector<GraspCandidatePtr>& grasp_candidates,
                            const planning_scene::PlanningScenePtr& planning_scene,
                            const robot_model::JointModelGroup* arm_jmg, const moveit::core::RobotStatePtr& seed_state,
                            bool filter_pregrasp = false, const std::string& target_object_id = "");
  /**
   * \brief Return grasps that are kinematically feasible
   * \param grasp_candidates - all possible grasps that this will test. this vector is returned modified
   * \param arm_jmg - the arm to solve the IK problem on
   * \param seed_state - A robot state to be used for IK. Ideally this will be close to the desired goal configuration.
   * \param filter_pregrasp - Whether to also check ik feasibility for the pregrasp position
   * \param visualize - visualize IK filtering
   * \param target_object_id - The name of the target grasp object in the planning scene if it exists
   * \return true on successful filter
   */
  virtual std::size_t filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                         const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
                                         const robot_model::JointModelGroup* arm_jmg,
                                         const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp,
                                         bool visualize, const std::string& grasp_target_object_id = "");

  virtual std::size_t filterGraspsHelper(std::vector<GraspCandidatePtr>& grasp_candidates,
                                         const planning_scene::PlanningScenePtr& planning_scene,
                                         const robot_model::JointModelGroup* arm_jmg,
                                         const moveit::core::RobotStatePtr& seed_state, bool filter_pregrasp,
                                         bool visualize, const std::string& grasp_target_object_id = "");

  /**
   * \brief Print grasp filtering statistics
   */
  virtual void printFilterStatistics(const std::vector<GraspCandidatePtr>& grasp_candidates) const;

  /**
   * \brief Method for checking part of the possible grasps list. MUST BE THREAD SAFE
   */
  virtual bool processCandidateGrasp(const IkThreadStructPtr& ik_thread_struct);

  /**
   * \brief Used for sorting an array of CandidateGrasps
   * \return true if A is less than B
   */
  static bool compareGraspScores(const GraspCandidatePtr& grasp_a, const GraspCandidatePtr& grasp_b)
  {
    // Determine if A or B has higher quality
    return (grasp_a->grasp_.grasp_quality > grasp_b->grasp_.grasp_quality);
  }

  /**
   * \brief Of an array of grasps, sort the valid ones from best score to worse score
   * \return false if no grasps remain
   */
  bool removeInvalidAndFilter(std::vector<GraspCandidatePtr>& grasp_candidates) const;

  /**
   * \brief Add a cutting plane filter for a shelf bin
   * \return true on success
   */
  bool addCuttingPlanesForBin(const Eigen::Isometry3d& world_to_bin, const Eigen::Isometry3d& bin_to_product,
                              const double& bin_width, const double& bin_height);

  /* \brief Set the ACM entry to ignore collisions between the ee_link_names and the object in the planning_scene */
  void setACMFingerEntry(const std::string& object_name, bool allowed, const std::vector<std::string>& ee_link_names,
                         const planning_scene::PlanningScenePtr& planning_scene);

protected:
  /**
   * \brief Filter grasps by cutting plane
   * \param grasp_candidates - all possible grasps that this will test. this vector is returned modified
   * \param filter_pose - pose of filter that will define cutting plane
   * \param plane - the cutting plane (XY, XZ, or YZ)
   * \param direction - which side of this plane to cut (+/- 1)
   * \return true if grasp is filtered by operation
   */
  bool filterGraspByPlane(GraspCandidatePtr& grasp_candidate, const Eigen::Isometry3d& filter_pose,
                          GraspParallelPlane plane, int direction) const;

  /**
   * \brief Filter grasps by desired orientation. Think of reaching into a small opening, you can only rotate your hand
   * a tiny amount and still grasp an object. If there's empty space behind an object, grasps behind the object aren't
   * removed by the cutting plane operations. We know we'll never get to them because they deviate too much from the
   * desired grasping pose... straight in.
   * \param grasp_candidates - a grasp candidate that this will test.
   * \param desired_pose - the desired grasp pose ( using standard grasping orientation )
   * \param max_angular_offset - maximum angle allowed between the grasp pose and the desired pose
   * \return true if grasp is filtered by operation
   */
  bool filterGraspByOrientation(GraspCandidatePtr& grasp_candidate, const Eigen::Isometry3d& desired_pose,
                                double max_angular_offset) const;

  /**
   * \brief Filter grasps by feasablity of the grasp pose.
   * \param grasp_candidates - a grasp candidate that this will test
   * \param grasp_ik_solution - the IK solution if one exists
   * \param ik_thread_struct - a struct containing the planning_scene
   * \return true if grasp is filtered by operation
   */
  bool filterGraspByGraspIK(const GraspCandidatePtr& grasp_candidate, std::vector<double>& grasp_ik_solution,
                            const IkThreadStructPtr& ik_thread_struct) const;

  /**
   * \brief Filter grasps by feasablity of the pre grasp pose.
   * \param grasp_candidates - a grasp candidate that this will test
   * \param grasp_ik_solution - the IK solution if one exists
   * \param ik_thread_struct - a struct containing the planning_scene
   * \return true if grasp is filtered by operation
   */
  bool filterGraspByPreGraspIK(const GraspCandidatePtr& grasp_candidate, std::vector<double>& pregrasp_ik_solution,
                               const IkThreadStructPtr& ik_thread_struct) const;

  /**
   * \brief Helper for the thread function to find IK solutions
   * \return true on success
   */
  bool findIKSolution(std::vector<double>& ik_solution, const IkThreadStructPtr& ik_thread_struct,
                      const GraspCandidatePtr& grasp_candidate,
                      const moveit::core::GroupStateValidityCallbackFn& constraint_fn) const;

  /**
   * \brief add a cutting plane
   * \param pose - pose describing the cutting plane
   * \param plane - which plane to use as the cutting plane
   * \param direction - on which side of the plane the grasps will be removed
   */
  void addCuttingPlane(const Eigen::Isometry3d& pose, GraspParallelPlane plane, int direction);

  /**
   * \brief Show all cutting planes that are currently enables
   * \return true on success
   */
  bool visualizeCuttingPlanes();

  /**
   * \brief clear all cutting planes
   */
  void clearCuttingPlanes();

  /**
   * \brief add a desired grasp orientation
   * \param pose - the desired grasping pose
   * \param max_angle_offset - maximum amount a generated grasp can deviate from the desired pose
   */
  void addDesiredGraspOrientation(const Eigen::Isometry3d& pose, double max_angle_offset);

  /**
   * \brief clear all desired orientations
   */
  void clearDesiredGraspOrientations();

  /**
   * \brief Show grasps after being filtered
   * \return true on success
   */
  bool visualizeGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates,
                       const moveit::core::JointModelGroup* arm_jmg);

  /**
   * \brief Show IK solutions of entire arm
   * \return true on success
   */
  bool visualizeIKSolutions(const std::vector<GraspCandidatePtr>& grasp_candidates,
                            const moveit::core::JointModelGroup* arm_jmg, double animation_speed);

  /**
   * \brief Show solutions of entire arm
   * \return true on success
   */
  bool visualizeCandidateGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates);

  /**
   * \brief Show an entire planning scene
   */
  void publishPlanningScene(const planning_scene::PlanningScenePtr& ps) const;

protected:
  // logging name
  const std::string name_;

  // Allow a writeable robot state
  robot_state::RobotStatePtr robot_state_;

  // Keep a robot state for every thread
  std::vector<robot_state::RobotStatePtr> robot_states_;

  // Threaded kinematic solvers
  std::map<std::string, std::vector<kinematics::KinematicsBaseConstPtr> > kin_solvers_;

  // Class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  // for rviz visualization of the planning scene
  ros::Publisher planning_scene_publisher_;

  // Number of degrees of freedom for the IK solver to find
  std::size_t num_variables_;

  // Time to allow IK solver to run
  double solver_timeout_;

  // Visualization levels
  bool collision_verbose_;
  bool statistics_verbose_;
  double collision_verbose_speed_;
  bool show_filtered_grasps_;
  bool show_filtered_arm_solutions_;
  bool show_cutting_planes_;
  double show_filtered_arm_solutions_speed_;
  double show_filtered_arm_solutions_pregrasp_speed_;
  bool show_grasp_filter_collision_if_failed_;

  // Shared node handle
  ros::NodeHandle nh_;

  // Cutting planes and orientation filter
  std::vector<CuttingPlanePtr> cutting_planes_;
  std::vector<DesiredGraspOrientationPtr> desired_grasp_orientations_;

};  // end of class

typedef std::shared_ptr<GraspFilter> GraspFilterPtr;
typedef std::shared_ptr<const GraspFilter> GraspFilterConstPtr;

}  // namespace moveit_grasps

#endif
