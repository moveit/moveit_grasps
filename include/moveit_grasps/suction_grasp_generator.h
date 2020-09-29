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

#ifndef MOVEIT_GRASPS__SUCTION_GRASP_GENERATOR_H_
#define MOVEIT_GRASPS__SUCTION_GRASP_GENERATOR_H_

#include <moveit_grasps/grasp_generator.h>
#include <moveit_grasps/suction_grasp_candidate.h>
#include <moveit_grasps/suction_grasp_scorer.h>
#include <moveit_grasps/suction_grasp_data.h>

namespace moveit_grasps
{
class SuctionGraspGenerator : public GraspGenerator
{
  friend class GraspGeneratorTest;

public:
  // Eigen requires 128-bit alignment for the Eigen::Vector2d's array (of 2 doubles).
  // With GCC, this is done with a attribute ((aligned(16))).
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * \brief Constructor
   */
  SuctionGraspGenerator(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, bool verbose = false);

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
                      const SuctionGraspDataPtr& grasp_data, std::vector<GraspCandidatePtr>& grasp_candidates);

  /**
   * \brief creates grasp messages from the generated grasp poses
   * \param grasp_pose_eef_mount - the grasp pose. (Note: this is the pose of the eef mount not the position of the tcp)
   * \param grasp_data data describing the end effector
   * \param object_pose - pose of object to grasp
   * \param object_size - size of object to grasp
   * \param grasp_candidates - output, list possible grasps with new grasp appended
   * \return true on success
   */
  bool addGrasp(const Eigen::Isometry3d& grasp_pose_eef_mount, const SuctionGraspDataPtr& grasp_data,
                const Eigen::Isometry3d& object_pose, const Eigen::Vector3d& object_size,
                std::vector<GraspCandidatePtr>& grasp_candidates);

  [[deprecated("Object_width no longer needs to be specified for suction grasps")]] bool
  addGrasp(const Eigen::Isometry3d& grasp_pose_eef_mount, const SuctionGraspDataPtr& grasp_data,
           const Eigen::Isometry3d& object_pose, const Eigen::Vector3d& object_size, double object_width,
           std::vector<GraspCandidatePtr>& grasp_candidates)
  {
    return addGrasp(grasp_pose_eef_mount, grasp_data, object_pose, object_size, grasp_candidates);
  }

  /**
   * \brief Setter for grasp score weights
   */
  void setGraspScoreWeights(const SuctionGraspScoreWeightsPtr& grasp_score_weights)
  {
    auto suction_grasp_score_weights = std::make_shared<SuctionGraspScoreWeights>(*grasp_score_weights);
    grasp_score_weights_ = std::dynamic_pointer_cast<GraspScoreWeights>(suction_grasp_score_weights);
  }

  /**
   * \brief Getter for grasp score weights
   */
  SuctionGraspScoreWeightsConstPtr getGraspScoreWeights()
  {
    return std::dynamic_pointer_cast<const SuctionGraspScoreWeights>(grasp_score_weights_);
  }

protected:
  /**
   * \brief Score the generated suction grasp poses
   * \param grasp_pose_tcp - the pose of the grasp
   * \param grasp_data - data describing the end effector
   * \param cuboid_pose - the pose of the object being grasped
   * \param object size - the extents of the object being grasped
   * \param suction_voxel_overlap - all voxels with a percentage of coverage above some cutoff
   * \return a score with positive being better
   */
  double scoreSuctionGrasp(const Eigen::Isometry3d& grasp_pose_tcp, const SuctionGraspDataPtr& grasp_data,
                           const Eigen::Isometry3d& cuboid_pose, const Eigen::Vector3d& object_size,
                           std::vector<double>& suction_voxel_overlap);

  /* \brief helper function to re-orient the cuboid center top grasp so it is as close as possible to the ideal grasp
   * orientation */
  void orientCuboidTowardsIdealTCP(Eigen::Isometry3d& cuboid_pose, double depth, double width, double height);

  /* \brief helper function for generating suction grasps */
  bool generateSuctionGrasps(const Eigen::Isometry3d& cuboid_top_pose, double depth, double width, double height,
                             const SuctionGraspDataPtr& grasp_data, std::vector<GraspCandidatePtr>& grasp_candidates);

public:
  // Visual debug settings
  bool debug_top_grasps_;
  bool show_grasp_overhang_;

};  // end of class

typedef std::shared_ptr<SuctionGraspGenerator> SuctionGraspGeneratorPtr;
typedef std::shared_ptr<const SuctionGraspGenerator> SuctionGraspGeneratorConstPtr;

}  // namespace moveit_grasps

#endif
