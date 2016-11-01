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

/* Author: Dave Coleman <dave@dav.ee>, Andy McEvoy
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
#include <moveit/macros/deprecation.h>

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
#include <math.h>
#include <limits>
#define _USE_MATH_DEFINES

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

// Class
class GraspGenerator
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles).
                                   // With GCC, this is done with a attribute ((aligned(16))).

      /**
       * \brief Constructor
       */
      GraspGenerator(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose = false);

  /**
   * \brief Create possible grasp positions around a cuboid
   * \param mesh_msg - model of object to grasp from perception
   * \param cuboid_pose - centroid of object to grasp in world frame
   * \param grasp_data data describing end effector
   * \param grasp_candidates possible grasps generated
   * \return true if successful
   */
  // bool generateGrasps(const shape_msgs::Mesh& mesh_msg, const Eigen::Affine3d& cuboid_pose,
  //                          const moveit_grasps::GraspDataPtr grasp_data, std::vector<GraspCandidatePtr>&
  //                          grasp_candidates);

  /**
   * \brief Create possible grasp positions around a cuboid
   * \param cuboid_pose - centroid of object to grasp in world frame
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \param grasp_data data describing end effector
   * \param grasp_candidates possible grasps generated
   * \return true if successful
   */
  bool generateGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width, double height,
                      const GraspDataPtr grasp_data, std::vector<GraspCandidatePtr>& grasp_candidates);

  /**
   * \brief Create grasp positions around one axis of a cuboid
   * \param cuboid_pose - centroid of object to grasp in world frame
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \param axis axis of cuboid to generate grasps around
   * \param grasp_data data describing end effector
   * \param grasp_candidates possible grasps generated
   * \param only_edge_grasps - set to true if object is too wide to grap the face in this axis
   * \return true if successful
   */
  bool generateCuboidAxisGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width, double height,
                                grasp_axis_t axis, const GraspDataPtr grasp_data,
                                std::vector<GraspCandidatePtr>& grasp_candidates, bool only_edge_grasps);

  /**
   * \brief helper function for adding grasps at corner of cuboid
   * \param pose - pose of the object to grasp
   * \param rotation_angles - rotation angles to go from cuboid pose to standard grasping pose
   * \param translation - translation to go from cuboid centroid to grasping location
   * \param corner_rotation - extra rotatation needed to align grasp pose as you move around the cuboid
   * \param num_radial_grasps - the number of grasps to generate around the corner
   * \param grasp_poses - list of grasp poses generated
   * \return the number of poses generated
   */
  std::size_t addCornerGraspsHelper(Eigen::Affine3d pose, double rotation_angles[3], Eigen::Vector3d translation,
                                    double corner_rotation, std::size_t num_radial_grasps,
                                    std::vector<Eigen::Affine3d>& grasp_poses);

  /**
   * \brief helper function for adding grasps along the face of a cuboid
   * \param pose - pose of the object to grasp
   * \param rotation_angles - rotation angles to go from cuboid pose to standard grasping pose
   * \param delta - distance to move away from cuboid at each step
   * \param translation - translation to go from cuboid centroid to grasping location
   * \param alignment_rotation - extra rotatation needed to align grasp pose as you move around the cuboid
   * \param num_grasps - the number of grasps to generate around the corner
   * \param grasp_poses - list of grasp poses generated
   * \return the number of poses generated
   */
  std::size_t addFaceGraspsHelper(Eigen::Affine3d pose, double rotation_angles[3], Eigen::Vector3d translation,
                                  Eigen::Vector3d delta, double alignment_rotation, std::size_t num_grasps,
                                  std::vector<Eigen::Affine3d>& grasp_poses);

  /**
   * \brief helper function for adding grasps along the edges of the cuboid
   * \param pose - pose of the object to grasp
   * \param rotation_angles - rotation angles to go from cuboid pose to standard grasping pose
   * \param delta - distance to move away from cuboid at each step
   * \param translation - translation to go from cuboid centroid to grasping location
   * \param alignment_rotation - extra rotatation needed to align grasp pose as you move around the cuboid
   * \param num_grasps - the number of grasps to generate around the corner
   * \param grasp_poses - list of grasp poses generated
   * \return the number of poses generated
   */
  std::size_t addEdgeGraspsHelper(Eigen::Affine3d cuboid_pose, double rotation_angles[3], Eigen::Vector3d translation,
                                  Eigen::Vector3d delta, double alignment_rotation, std::size_t num_grasps,
                                  std::vector<Eigen::Affine3d>& grasp_poses, double corner_rotation);

  /**
   * \brief helper function for determining if the grasp will intersect the cuboid
   * \param cuboid_pose - centroid of object to grasp in world frame
   * \param depth - size of cuboid along x axis
   * \param width - size of cuboid along y axis
   * \param height - size of cuboid along z axis
   * \param grasp_pose - pose of grasp
   * \param grasp_data - data describing end effector
   * \return true if the grasp intersects the cuboid
   */
  bool graspIntersectionHelper(Eigen::Affine3d cuboid_pose, double depth, double width, double height,
                               Eigen::Affine3d grasp_pose, const GraspDataPtr grasp_data);

  /**
   * \brief helper function to test intersection of a line with a plane
   * \param t - parametric distance along grasp line
   * \param u1, v1, u2, v2 - (u,v) coordinates of the line
   * \param a, b - length and width of the plane area in which to test for intersection
   * \param u, v - location of intersection
   * \return true if the line intersects the plane
   */
  bool intersectionHelper(double t, double u1, double v1, double u2, double v2, double a, double b, double& u,
                          double& v);

  /**
   * \brief creates grasp messages from the generated grasp poses
   * \param pose - the grasp pose
   * \param grasp_data data describing the end effector
   * \param grasp_candidates - list possible grasps
   * \param object_pose - pose of object to grasp
   * \return true on success
   */
  bool addGrasp(const Eigen::Affine3d& pose, const GraspDataPtr grasp_data,
                std::vector<GraspCandidatePtr>& grasp_candidates, const Eigen::Affine3d& object_pose,
                double object_width);

  /**
   * \brief Score the generated grasp poses
   * \param grasp_pose - the pose of the grasp
   * \param grasp_data - data describing the end effector
   * \param object_pose - the pose of the object being grasped
   * \param percent_open - percentage that the grippers are open. 0.0 -> grippers are at object width + padding
   * \return
   */
  double scoreGrasp(const Eigen::Affine3d& grasp_pose, const GraspDataPtr grasp_data, const Eigen::Affine3d object_pose,
                    double percent_open);

  /**
   * \brief Get the grasp direction vector relative to the world frame
   * \param grasp
   * \param name of parent link
   * \return the approach direction
   */
  static Eigen::Vector3d getPreGraspDirection(const moveit_msgs::Grasp& grasp, const std::string& ee_parent_link);
  //  static Eigen::Vector3d getPostGraspDirection(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

  /**
   * \brief Using an input grasp description, get the pregrasp pose
   * \param grasp
   * \param name of parent link
   * \return pregrasp pose
   */
  static geometry_msgs::PoseStamped getPreGraspPose(const moveit_msgs::Grasp& grasp, const std::string& ee_parent_link);
  // static geometry_msgs::PoseStamped getPostGraspPose(const moveit_msgs::Grasp &grasp, const std::string
  // &ee_parent_link);

  /**
   * \brief Helper to convert a robot-specific grasp to an arrow pointed in the right direction
   * \param grasp - the grasp to show
   * \param arm - the planning group of the arm we want to display
   * \return true on success
   */
  void publishGraspArrow(geometry_msgs::Pose grasp, const GraspDataPtr grasp_data,
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
  Eigen::Affine3d getIdealGraspPose()
  {
    return ideal_grasp_pose_;
  }

  /**
   * \brief Setter for ideal grasp pose for scoring
   */
  void setIdealGraspPose(Eigen::Affine3d ideal_pose)
  {
    ideal_grasp_pose_ = ideal_pose;
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
  Eigen::Affine3d ideal_grasp_pose_;

private:
  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Display more output both in console
  bool verbose_;

  // Shared node handle
  ros::NodeHandle nh_;

  // Transform from frame of box to global frame
  Eigen::Affine3d object_global_transform_;

  // Visualization levels
  bool show_grasp_arrows_;
  double show_grasp_arrows_speed_;

  bool show_prefiltered_grasps_;
  double show_prefiltered_grasps_speed_;

  double min_grasp_distance_, max_grasp_distance_;
  Eigen::Vector3d min_translations_, max_translations_;

  double depth_score_weight_;
  double width_score_weight_;
  double height_score_weight_;
  double orientation_x_score_weight_;
  double orientation_y_score_weight_;
  double orientation_z_score_weight_;
  double translation_x_score_weight_;
  double translation_y_score_weight_;
  double translation_z_score_weight_;

  // bounding_box::BoundingBox bounding_box_;

};  // end of class

typedef boost::shared_ptr<GraspGenerator> GraspGeneratorPtr;
typedef boost::shared_ptr<const GraspGenerator> GraspGeneratorConstPtr;

}  // namespace

#endif
