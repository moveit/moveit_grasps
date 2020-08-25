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

#ifndef MOVEIT_GRASPS__TWO_FINGER_GRASP_GENERATOR_H_
#define MOVEIT_GRASPS__TWO_FINGER_GRASP_GENERATOR_H_

#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/grasp_candidate.h>
#include <moveit_grasps/two_finger_grasp_scorer.h>
#include <moveit_grasps/two_finger_grasp_data.h>

// Testing
#include <gtest/gtest.h>

namespace moveit_grasps
{
struct TwoFingerGraspCandidateConfig
{
  TwoFingerGraspCandidateConfig();
  void enableAllGraspTypes();
  void enableAllGraspAxes();
  void enableAll();
  void disableAllGraspTypes();
  void disableAllGraspAxes();
  void disableAll();

  ///////////////////////////////
  // Finger Gripper config values
  ///////////////////////////////
  bool enable_corner_grasps_;
  bool enable_face_grasps_;
  bool enable_variable_angle_grasps_;
  bool enable_edge_grasps_;
  bool generate_x_axis_grasps_;
  bool generate_y_axis_grasps_;
  bool generate_z_axis_grasps_;
};

class TwoFingerGraspGenerator : public GraspGenerator
{
public:
  // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles).
  // With GCC, this is done with a attribute ((aligned(16))).
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief Constructor
   */
  TwoFingerGraspGenerator(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, bool verbose = false);

  /* brief sets internal grasp_candidate_config_ variable
   * \param grasp_candidate_config - a config describing the grasps to be generated
   */
  void setGraspCandidateConfig(const TwoFingerGraspCandidateConfig& grasp_candidate_config);

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
  bool generateGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width, double height,
                      const GraspDataPtr& grasp_data, std::vector<GraspCandidatePtr>& grasp_candidates) override;

  bool generateGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width, double height,
                      const TwoFingerGraspDataPtr& grasp_data, std::vector<GraspCandidatePtr>& grasp_candidates);

  /**
   * \brief Setter for grasp score weights
   */
  void setGraspScoreWeights(const TwoFingerGraspScoreWeightsPtr& grasp_score_weights)
  {
    auto two_finger_grasp_score_weights = std::make_shared<TwoFingerGraspScoreWeights>(*grasp_score_weights);
    grasp_score_weights_ = std::dynamic_pointer_cast<GraspScoreWeights>(two_finger_grasp_score_weights);
  }

  /**
   * \brief Setter for grasp score weights
   */
  const TwoFingerGraspScoreWeightsPtr getGraspScoreWeights()
  {
    return std::dynamic_pointer_cast<TwoFingerGraspScoreWeights>(grasp_score_weights_);
  }

protected:
  /**
   * \brief creates grasp messages from the generated grasp poses
   * \param grasp_pose_eef_mount - the grasp pose. (Note: this is the pose of the eef mount not the position of the tcp)
   * \param grasp_data data describing the end effector
   * \param object_pose - pose of object to grasp
   * \param object_size - size of object to grasp
   * \param object_width - In the case of finger grippers, the width of the object in the dimension betwen the fingers
   * \param grasp_candidates - list possible grasps with new grasp appended
   * \return true on success
   */
  bool addGrasp(const Eigen::Isometry3d& grasp_pose_eef_mount, const TwoFingerGraspDataPtr& grasp_data,
                const Eigen::Isometry3d& object_pose, const Eigen::Vector3d& object_size, double object_width,
                std::vector<GraspCandidatePtr>& grasp_candidates);

  /**
   * \brief Create grasp positions around one axis of a cuboid
   * \param cuboid_pose:      centroid of object to grasp in world frame
   * \param depth:            length of cuboid along local x-axis
   * \param width:            length of cuboid along local y-axis
   * \param height:           length of cuboid along local z-axis
   * \param axis:             axis of cuboid to generate grasps along
   * \param grasp_data:       data describing end effector
   * \param grasp_candidates: possible grasps generated
   * \param only_edge_grasps: set to true if object is too wide to grap the face in this axis
   * \return true if successful
   */
  bool generateCuboidAxisGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width, double height,
                                grasp_axis_t axis, const TwoFingerGraspDataPtr& grasp_data,
                                const TwoFingerGraspCandidateConfig& grasp_candidate_config,
                                std::vector<GraspCandidatePtr>& grasp_candidates);

  /**
   * \brief helper function for adding grasps at corner of cuboid
   * \param pose - pose of the object to grasp
   * \param rotation_angles - rotation angles to go from cuboid pose to standard grasping pose
   * \param translation - translation to go from cuboid centroid to grasping location
   * \param corner_rotation - extra rotatation needed to align grasp pose as you move around the cuboid
   * \param num_radial_grasps - the number of grasps to generate around the corner
   * \param grasp_poses_tcp - list of grasp poses generated
   * \return the number of poses generated
   */
  std::size_t addCornerGraspsHelper(const Eigen::Isometry3d& pose, double rotation_angles[3],
                                    const Eigen::Vector3d& translation, double corner_rotation,
                                    std::size_t num_radial_grasps, EigenSTL::vector_Isometry3d& grasp_poses_tcp);

  /**
   * \brief helper function for adding grasps along the face of a cuboid
   * \param pose - pose of the object to grasp
   * \param rotation_angles - rotation angles to go from cuboid pose to standard grasping pose
   * \param delta - distance to move away from cuboid at each step
   * \param translation - translation to go from cuboid centroid to grasping location
   * \param alignment_rotation - extra rotatation needed to align grasp pose as you move around the cuboid
   * \param num_grasps - the number of grasps to generate around the corner
   * \param grasp_poses_tcp - list of grasp poses generated
   * \return the number of poses generated
   */
  std::size_t addFaceGraspsHelper(const Eigen::Isometry3d& pose, double rotation_angles[3],
                                  const Eigen::Vector3d& translation, const Eigen::Vector3d& delta,
                                  double alignment_rotation, std::size_t num_grasps,
                                  EigenSTL::vector_Isometry3d& grasp_poses_tcp);

  /**
   * \brief helper function for adding grasps along the edges of the cuboid
   * \param pose - pose of the object to grasp
   * \param rotation_angles - rotation angles to go from cuboid pose to standard grasping pose
   * \param delta - distance to move away from cuboid at each step
   * \param translation - translation to go from cuboid centroid to grasping location
   * \param alignment_rotation - extra rotatation needed to align grasp pose as you move around the cuboid
   * \param num_grasps - the number of grasps to generate around the corner
   * \param grasp_poses_tcp - list of grasp poses generated
   * \return the number of poses generated
   */
  std::size_t addEdgeGraspsHelper(const Eigen::Isometry3d& cuboid_pose, double rotation_angles[3],
                                  const Eigen::Vector3d& translation, const Eigen::Vector3d& delta,
                                  double alignment_rotation, std::size_t num_grasps,
                                  EigenSTL::vector_Isometry3d& grasp_poses_tcp, double corner_rotation);

  /**
   * \brief helper function for determining if the grasp will intersect the cuboid
   * \param cuboid_pose - centroid of object to grasp in world frame
   * \param depth - size of cuboid along x axis
   * \param width - size of cuboid along y axis
   * \param height - size of cuboid along z axis
   * \param grasp_pose_tcp - pose of grasp
   * \param grasp_data - data describing end effector
   * \return true if the grasp intersects the cuboid
   */
  bool graspIntersectionHelper(const Eigen::Isometry3d& cuboid_pose, double depth, double width, double height,
                               const Eigen::Isometry3d& grasp_pose_tcp, const TwoFingerGraspDataPtr& grasp_data);

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
   * \brief Score the generated finger grasp poses
   * \param grasp_pose_tcp - the grasp pose of the tcp
   * \param grasp_data - data describing the end effector
   * \param object_pose - the pose of the object being grasped
   * \param percent_open - percentage that the grippers are open. 0.0 -> grippers are at object width + padding
   * \return a score with positive being better
   */
  double scoreFingerGrasp(const Eigen::Isometry3d& grasp_pose_tcp, const TwoFingerGraspDataPtr& grasp_data,
                          const Eigen::Isometry3d& object_pose, double percent_open);
  bool
  generateFingerGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width, double height,
                       const TwoFingerGraspDataPtr& grasp_data, std::vector<GraspCandidatePtr>& grasp_candidates,
                       const TwoFingerGraspCandidateConfig& grasp_candidate_config = TwoFingerGraspCandidateConfig());

protected:
  TwoFingerGraspCandidateConfig grasp_candidate_config_;

  // Tests
  FRIEND_TEST(TwoFingerGraspGeneratorTest, GenerateFaceGrasps);
  FRIEND_TEST(TwoFingerGraspGeneratorTest, GenerateEdgeGrasps);
  FRIEND_TEST(TwoFingerGraspGeneratorTest, GenerateCornerGrasps);

};  // end of class

typedef std::shared_ptr<TwoFingerGraspGenerator> TwoFingerGraspGeneratorPtr;
typedef std::shared_ptr<const TwoFingerGraspGenerator> TwoFingerGraspGeneratorConstPtr;

}  // namespace moveit_grasps

#endif
