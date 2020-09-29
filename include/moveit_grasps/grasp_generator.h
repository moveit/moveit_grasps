/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, University of Colorado, Boulder
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

/* Author: Dave Coleman <dave@picknik.ai>, Andy McEvoy
   Desc:   Generates geometric grasps for cuboids and blocks, not using physics or contact wrenches
*/

#ifndef MOVEIT_GRASPS__GRASP_GENERATOR_H_
#define MOVEIT_GRASPS__GRASP_GENERATOR_H_

// ROS
#include <ros/ros.h>

// TF
#include <tf_conversions/tf_eigen.h>

// Msgs
#include <geometry_msgs/PoseArray.h>

// MoveIt
#include <moveit_msgs/Grasp.h>

// geometric_shapes
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/bodies.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <eigen_conversions/eigen_msg.h>

// Visualization
#include <moveit_visual_tools/moveit_visual_tools.h>

// moveit_grasps
#include <moveit_grasps/grasp_candidate.h>
#include <moveit_grasps/grasp_scorer.h>

// bounding_box
//#include <bounding_box/bounding_box.h>

// C++
#include <cstdlib>
#include <string>
#include <cmath>
#include <limits>

#include <moveit_grasps/grasp_data.h>

namespace moveit_grasps
{
static const double RAD2DEG = 57.2957795;
static const double MIN_GRASP_DISTANCE = 0.001;  // m between grasps

// Grasp axis orientation
enum grasp_axis_t
{
  X_AXIS,
  Y_AXIS,
  Z_AXIS
};

class GraspGenerator
{
public:
  // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles).
  // With GCC, this is done with a attribute ((aligned(16))).
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief Constructor
   */
  GraspGenerator(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, bool verbose = false);

  /**
   * \brief Create possible grasp positions around a cuboid
   * \param cuboid_pose - centroid of object to grasp in world frame
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \param grasp_data data describing end effector
   * \param grasp_candidate_config parameter for selectively enabling and disabling different grasp types
   * \param grasp_candidates possible grasps generated
   * \return true if successful
   */
  virtual bool generateGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width, double height,
                              const GraspDataPtr& grasp_data, std::vector<GraspCandidatePtr>& grasp_candidates) = 0;

  /**
   * \brief Get the grasp direction vector relative to the world frame
   * \param grasp
   * \param name of parent link
   * \return the approach direction
   */
  static Eigen::Vector3d getPreGraspDirection(const moveit_msgs::Grasp& grasp, const std::string& ee_parent_link);
  //  static Eigen::Vector3d getPostGraspDirection(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

  /**
   * \brief Using an input grasp description, get the pregrasp pose. The pregrasp pose is the grasp pose translated
   * backwards in the grasp frame away from the object being grasped.
   * \param grasp
   * \param name of parent link
   * \return pregrasp pose
   */
  static geometry_msgs::PoseStamped getPreGraspPose(const GraspCandidatePtr& grasp_candidate,
                                                    const std::string& ee_parent_link);
  /**
   * \brief Compute the pre-grasp, grasp, lift and retreat poses for a grasp candidate
   * \param grasp_candidate - the grasp candidate
   * \param grasp_waypoints - a reference to a vector that will be populated with the pre-grasp, grasp, lift and retreat
   * poses in that order.
   */
  static void getGraspWaypoints(const GraspCandidatePtr& grasp_candidate, EigenSTL::vector_Isometry3d& grasp_waypoints);

  /**
   * \brief Helper to convert a robot-specific grasp to an arrow pointed in the right direction
   * \param grasp - the grasp to show
   * \param arm - the planning group of the arm we want to display
   * \return true on success
   */
  void publishGraspArrow(const geometry_msgs::Pose& grasp, const GraspDataPtr& grasp_data,
                         const rviz_visual_tools::colors& color, double approach_length = 0.1);

  /**
   * \brief Getter for Verbose
   */
  bool getVerbose()
  {
    return verbose_;
  }

  /**
   * \brief Getter for ideal grasp pose
   */
  Eigen::Isometry3d getIdealTCPGraspPose()
  {
    return ideal_grasp_pose_;
  }

  /**
   * \brief Setter for ideal grasp pose for scoring
   */
  void setIdealTCPGraspPose(Eigen::Isometry3d ideal_pose)
  {
    ideal_grasp_pose_ = ideal_pose;
  }

  /**
   * \brief Setter for the roll pitch yall ideal grasp pose for scoring
   */
  void setIdealTCPGraspPoseRPY(const std::vector<double>& ideal_grasp_orientation_rpy);

  /**
   * \brief Setter for grasp score weights
   */
  void setGraspScoreWeights(const GraspScoreWeightsPtr& grasp_score_weights)
  {
    grasp_score_weights_ = std::make_shared<GraspScoreWeights>(*grasp_score_weights);
  }

  /**
   * \brief Setter for grasp score weights
   */
  const GraspScoreWeightsPtr& getGraspScoreWeights()
  {
    return grasp_score_weights_;
  }

  /**
   * \brief Setter for Verbose
   */
  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }

  /**
   * \brief Visualize animated grasps
   * \return true on success
   */
  bool visualizeAnimatedGrasps(const std::vector<GraspCandidatePtr>& grasp_candidates,
                               const moveit::core::JointModelGroup* ee_jmg, double animation_speed);

  // Ideal grasp pose for scoring purposes
  Eigen::Isometry3d ideal_grasp_pose_;

protected:
  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Display more output both in console
  bool verbose_;

  // Visual debug settings
  bool show_prefiltered_grasps_;
  double show_prefiltered_grasps_speed_;

  // Shared node handle
  ros::NodeHandle nh_;

  // Transform from frame of box to global frame
  Eigen::Isometry3d object_global_transform_;

  double min_grasp_distance_, max_grasp_distance_;
  Eigen::Vector3d min_translations_, max_translations_;

  GraspScoreWeightsPtr grasp_score_weights_;

};  // end of class

typedef std::shared_ptr<GraspGenerator> GraspGeneratorPtr;
typedef std::shared_ptr<const GraspGenerator> GraspGeneratorConstPtr;

}  // namespace moveit_grasps

#endif
