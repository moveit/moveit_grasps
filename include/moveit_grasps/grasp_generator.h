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

// Author: Dave Coleman
// Desc:   Generates grasps for a cube

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
static const double MIN_GRASP_DISTANCE = 0.001; //m between grasps

// Grasp axis orientation
enum grasp_axis_t {X_AXIS, Y_AXIS, Z_AXIS};
enum grasp_parallel_plane{XY, XZ, YZ};

// Class
class GraspGenerator
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles). With GCC, this is done with a attribute ((aligned(16))).

  /**
   * \brief Constructor
   */
  GraspGenerator(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose = false);

  /**
   * \brief Create possible grasp positions around a cuboid 
   * \param mesh_msg - model of object to grasp from perception
   * \param cuboid_pose pose of cuboid 
   * \param grasp_data data describing end effector
   * \param possible_grasps possible grasps generated
   * \return true if successful
   */
  bool generateGrasps(const shape_msgs::Mesh& mesh_msg, const Eigen::Affine3d& cuboid_pose,
                            double max_grasp_size, const moveit_grasps::GraspDataPtr grasp_data,
                            std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Create possible grasp positions around a cuboid 
   * \param cuboid_pose pose of cuboid 
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \param grasp_data data describing end effector
   * \param possible_grasps possible grasps generated
   * \return true if successful
   */
  bool generateGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width,double height, 
                            double max_grasp_size, const GraspDataPtr grasp_data, 
                            std::vector<moveit_msgs::Grasp>& possible_grasps);
  
  /**
   * \brief Create grasp positions around one axis of a cuboid
   * \param cuboid_pose pose of cuboid 
   * \param depth length of cuboid along local x-axis
   * \param width length of cuboid along local y-axis
   * \param height length of cuboid along local z-axis
   * \param axis axis of cuboid to generate grasps around
   * \param grasp_data data describing end effector
   * \param possible_grasps possible grasps generated
   * \return true if successful
   */
  bool generateCuboidAxisGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width, double height, 
                                grasp_axis_t axis, const GraspDataPtr grasp_data, 
                                std::vector<moveit_msgs::Grasp>& possible_grasps);

  std::size_t addCornerGraspsHelper(Eigen::Affine3d pose, double rotation_angles[3], Eigen::Vector3d translation,
                                    double corner_rotation, int num_radial_grasps, 
                                    std::vector<Eigen::Affine3d>& grasp_poses);

  std::size_t addFaceGraspsHelper(Eigen::Affine3d pose, double rotation_angles[3], Eigen::Vector3d translation,
                                  Eigen::Vector3d delta, double alignment_rotation, int num_grasps,
                                  std::vector<Eigen::Affine3d>& grasp_poses);

  bool graspIntersectionHelper(Eigen::Affine3d cuboid_pose, double depth, double width, double height,
                               Eigen::Affine3d grasp_pose);

  bool intersectionHelper(double t, double u1, double v1, double u2, double v2, double a, double b);

  void addGrasp(const Eigen::Affine3d& pose, std::vector<moveit_msgs::Grasp>& possible_grasps);

  /**
   * \brief Get the grasp direction vector relative to the world frame
   * \param grasp 
   * \param name of parent link
   * \return the approach direction
   */
  static Eigen::Vector3d getPreGraspDirection(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

  /**
   * \brief Using an input grasp description, get the pregrasp pose
   * \param grasp 
   * \param name of parent link
   * \return pregrasp pose
   */
  static geometry_msgs::PoseStamped getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link);

  /**
   * \brief Helper to convert a robot-specific grasp to an arrow pointed in the right direction
   * \param grasp - the grasp to show
   * \param arm - the planning group of the arm we want to display
   * \return true on success
   */
  void publishGraspArrow(geometry_msgs::Pose grasp, const GraspDataPtr grasp_data, const rviz_visual_tools::colors &color, double approach_length = 0.1);

  /**
   * \brief get the bounding box for a mesh
   *
   */
  bool getBoundingBoxFromMesh(const shape_msgs::Mesh& mesh_msg, Eigen::Affine3d& cuboid_pose, 
                              double& depth, double& width, double& height);
                                            
  /**
   * \brief Getter for Verbose
   */ 
  bool getVerbose()
  {
    return verbose_;
  }

  /**
   * \brief Setter for delta between grasps
   */
  void setGraspDelta(double m_between_grasps)
  {
   m_between_grasps_ = m_between_grasps;
  }

  void setGraspDepthDelta(double m_between_depth_grasps)
  {
    m_between_depth_grasps_ = m_between_depth_grasps;
  }
  
  /**
   * \brief Setter for Verbose
   */
  void setVerbose(bool verbose)
  {
    verbose_ = verbose;
  }
  
private:

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // Transform from frame of box to global frame
  Eigen::Affine3d object_global_transform_;

  // Display more output both in console and in Rviz (with arrows and markers)
  bool verbose_;

  // Discretization of grasps
  double m_between_grasps_;
  double m_between_depth_grasps_;

  // Visualization levels
  bool show_grasp_arrows_;
  double show_grasp_arrows_speed_;

  bool show_prefiltered_grasps_;
  double show_prefiltered_grasps_speed_;

  // Shared node handle
  ros::NodeHandle nh_;

}; // end of class

typedef boost::shared_ptr<GraspGenerator> GraspGeneratorPtr;
typedef boost::shared_ptr<const GraspGenerator> GraspGeneratorConstPtr;

} // namespace

#endif
