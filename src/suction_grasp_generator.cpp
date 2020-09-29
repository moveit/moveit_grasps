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

/* Author: Dave Coleman <dave@picknik.ai>, Andy McEvoy
   Desc:   Generates geometric grasps for cuboids and blocks, not using physics or contact wrenches
*/

#include <moveit_grasps/suction_grasp_generator.h>
#include <moveit_grasps/suction_grasp_candidate.h>
#include <moveit_grasps/grasp_filter.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps
{
// Constructor
SuctionGraspGenerator::SuctionGraspGenerator(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools, bool verbose)
  : GraspGenerator(visual_tools, verbose)
{
  auto suction_grasp_score_weights = std::make_shared<SuctionGraspScoreWeights>();
  grasp_score_weights_ = std::dynamic_pointer_cast<GraspScoreWeights>(suction_grasp_score_weights);

  // Load visulization settings
  const std::string parent_name = "grasps";  // for namespacing logging messages
  std::size_t error = 0;

  error += !rosparam_shortcuts::get(parent_name, nh_, "debug_top_grasps", debug_top_grasps_);
  error += !rosparam_shortcuts::get(parent_name, nh_, "show_grasp_overhang", show_grasp_overhang_);

  // Load scoring weights
  rosparam_shortcuts::shutdownIfError(parent_name, error);
}

bool SuctionGraspGenerator::addGrasp(const Eigen::Isometry3d& grasp_pose_eef_mount,
                                     const SuctionGraspDataPtr& grasp_data, const Eigen::Isometry3d& object_pose,
                                     const Eigen::Vector3d& object_size,
                                     std::vector<GraspCandidatePtr>& grasp_candidates)
{
  // Transform the grasp pose eef mount to the tcp grasp pose
  Eigen::Isometry3d grasp_pose_tcp = grasp_pose_eef_mount * grasp_data->tcp_to_eef_mount_.inverse();

  // The new grasp
  moveit_msgs::Grasp new_grasp;

  // Approach and retreat - aligned with eef to grasp transform
  // set pregrasp
  moveit_msgs::GripperTranslation pre_grasp_approach;
  new_grasp.pre_grasp_approach.direction.header.stamp = ros::Time::now();
  new_grasp.pre_grasp_approach.desired_distance = grasp_data->grasp_max_depth_ + grasp_data->approach_distance_desired_;
  new_grasp.pre_grasp_approach.min_distance = 0;  // NOT IMPLEMENTED
  new_grasp.pre_grasp_approach.direction.header.frame_id = grasp_data->parent_link_->getName();

  Eigen::Vector3d grasp_approach_vector = -1 * grasp_data->tcp_to_eef_mount_.translation();
  grasp_approach_vector = grasp_approach_vector / grasp_approach_vector.norm();

  new_grasp.pre_grasp_approach.direction.vector.x = grasp_approach_vector.x();
  new_grasp.pre_grasp_approach.direction.vector.y = grasp_approach_vector.y();
  new_grasp.pre_grasp_approach.direction.vector.z = grasp_approach_vector.z();

  // set postgrasp
  moveit_msgs::GripperTranslation post_grasp_retreat;
  new_grasp.post_grasp_retreat.direction.header.stamp = ros::Time::now();
  new_grasp.post_grasp_retreat.desired_distance = grasp_data->grasp_max_depth_ + grasp_data->retreat_distance_desired_;
  new_grasp.post_grasp_retreat.min_distance = 0;  // NOT IMPLEMENTED
  new_grasp.post_grasp_retreat.direction.header.frame_id = grasp_data->parent_link_->getName();
  new_grasp.post_grasp_retreat.direction.vector.x = -1 * grasp_approach_vector.x();
  new_grasp.post_grasp_retreat.direction.vector.y = -1 * grasp_approach_vector.y();
  new_grasp.post_grasp_retreat.direction.vector.z = -1 * grasp_approach_vector.z();

  // set grasp pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data->base_link_;

  // name the grasp
  static std::size_t grasp_id = 0;
  new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
  grasp_id++;

  tf::poseEigenToMsg(grasp_pose_eef_mount, grasp_pose_msg.pose);
  new_grasp.grasp_pose = grasp_pose_msg;

  // set grasp postures e.g. hand closed
  new_grasp.grasp_posture = grasp_data->grasp_posture_;

  std::vector<double> suction_voxel_overlap;
  new_grasp.grasp_quality =
      scoreSuctionGrasp(grasp_pose_tcp, grasp_data, object_pose, object_size, suction_voxel_overlap);

  auto suction_grasp_candidate = std::make_shared<SuctionGraspCandidate>(new_grasp, grasp_data, object_pose);
  suction_grasp_candidate->setSuctionVoxelOverlap(suction_voxel_overlap);

  grasp_candidates.push_back(suction_grasp_candidate);
  return true;
}

double SuctionGraspGenerator::scoreSuctionGrasp(const Eigen::Isometry3d& grasp_pose_tcp,
                                                const SuctionGraspDataPtr& grasp_data,
                                                const Eigen::Isometry3d& cuboid_pose,
                                                const Eigen::Vector3d& object_size,
                                                std::vector<double>& suction_voxel_overlap)
{
  static const std::string logger_name = "grasp_generator.scoreGrasp";
  if (getVerbose())
  {
    ROS_DEBUG_STREAM_NAMED(logger_name,
                           "Scoring grasp at: \n\tpose:  ("
                               << grasp_pose_tcp.translation().x() << ",\t" << grasp_pose_tcp.translation().y() << ",\t"
                               << grasp_pose_tcp.translation().z() << ")\t("
                               << grasp_pose_tcp.rotation().eulerAngles(0, 1, 2)(0) << ",\t"
                               << grasp_pose_tcp.rotation().eulerAngles(0, 1, 2)(1) << ",\t"
                               << grasp_pose_tcp.rotation().eulerAngles(0, 1, 2)(2) << ")\n\tideal: ("
                               << ideal_grasp_pose_.translation().x() << ",\t" << ideal_grasp_pose_.translation().y()
                               << ",\t" << ideal_grasp_pose_.translation().z() << ")\t("
                               << ideal_grasp_pose_.rotation().eulerAngles(0, 1, 2)(0) << ",\t"
                               << ideal_grasp_pose_.rotation().eulerAngles(0, 1, 2)(1) << ",\t"
                               << ideal_grasp_pose_.rotation().eulerAngles(0, 1, 2)(2) << ")");
  }

  // get portion of score based on the orientation
  Eigen::Isometry3d ideal_grasp_tcp = getIdealTCPGraspPose();
  // Move the ideal top grasp to the box location
  ideal_grasp_tcp.translation() = cuboid_pose.translation();
  Eigen::Vector3d orientation_scores = SuctionGraspScorer::scoreRotationsFromDesired(grasp_pose_tcp, ideal_grasp_tcp);

  // get portion of score based on the translation
  Eigen::Vector3d translation_scores = SuctionGraspScorer::scoreGraspTranslation(grasp_pose_tcp, ideal_grasp_tcp);

  // Score suction grasp overhang
  double suction_overlap_score =
      SuctionGraspScorer::scoreSuctionVoxelOverlap(grasp_pose_tcp, grasp_data, cuboid_pose, object_size,
                                                   suction_voxel_overlap,
                                                   (show_grasp_overhang_ ? visual_tools_ : nullptr));

  double total_score = 0;
  auto suction_grasp_score_weights = std::dynamic_pointer_cast<SuctionGraspScoreWeights>(grasp_score_weights_);
  if (suction_grasp_score_weights)
  {
    total_score = suction_grasp_score_weights->computeScore(orientation_scores, translation_scores,
                                                            suction_overlap_score, getVerbose());
  }
  else
  {
    ROS_WARN_NAMED(logger_name, "Failed to cast grasp_score_weights_ as SuctionGraspScoreWeights. continuing without "
                                "finger specific scores");
    total_score = grasp_score_weights_->computeScore(orientation_scores, translation_scores, getVerbose());
  }

  return total_score;
}

bool SuctionGraspGenerator::generateGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                           double height, const moveit_grasps::GraspDataPtr& grasp_data,
                                           std::vector<GraspCandidatePtr>& grasp_candidates)
{
  auto suction_grasp_data = std::dynamic_pointer_cast<SuctionGraspData>(grasp_data);
  if (!suction_grasp_data)
  {
    ROS_ERROR_STREAM_NAMED("grasp_generator", "grasp_data is not castable to SuctionGraspData. Make sure you are using "
                                              "the child class");
    return false;
  }
  return generateGrasps(cuboid_pose, depth, width, height, suction_grasp_data, grasp_candidates);
}

bool SuctionGraspGenerator::generateGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                           double height, const moveit_grasps::SuctionGraspDataPtr& grasp_data,
                                           std::vector<GraspCandidatePtr>& grasp_candidates)
{
  bool result = generateSuctionGrasps(cuboid_pose, depth, width, height, grasp_data, grasp_candidates);

  if (result)
    std::sort(grasp_candidates.begin(), grasp_candidates.end(), GraspFilter::compareGraspScores);

  return result;
}

// X -> Depth
// Y -> Width
// Z -> Height
void SuctionGraspGenerator::orientCuboidTowardsIdealTCP(Eigen::Isometry3d& cuboid_pose_fixed, double depth,
                                                        double width, double height)
{
  // Move the ideal grasp pose to the center of the  box
  Eigen::Isometry3d ideal_grasp_tcp = getIdealTCPGraspPose();
  ideal_grasp_tcp.translation() = cuboid_pose_fixed.translation();

  if (debug_top_grasps_)
  {
    visual_tools_->publishAxis(cuboid_pose_fixed, rviz_visual_tools::SMALL, "cuboid_pose_fixed");
    visual_tools_->publishAxis(ideal_grasp_tcp, rviz_visual_tools::SMALL, "ideal_grasp_tcp");
    visual_tools_->trigger();
  }

  if (debug_top_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "cuboid_direction:\n" << cuboid_pose_fixed.rotation() << "\n");
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "ideal_grasp_tcp:\n" << ideal_grasp_tcp.rotation() << "\n");
    visual_tools_->publishWireframeCuboid(cuboid_pose_fixed, depth, width, height, rviz_visual_tools::YELLOW);
    visual_tools_->trigger();
    ros::Duration(1).sleep();
    // visual_tools_->prompt("start config");
  }

  // If the ideal top grasp Z axis is in the opposite direction of the top pose then we rotate around X to flip the
  // orientation vector
  double dot_prodZ = (cuboid_pose_fixed.rotation() * Eigen::Vector3d::UnitZ())
                         .dot(ideal_grasp_tcp.rotation() * Eigen::Vector3d::UnitZ());
  if (dot_prodZ < 0)
  {
    ROS_DEBUG_NAMED("grasp_generator", "flipping Z");
    cuboid_pose_fixed = cuboid_pose_fixed * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "New cuboid_direction:\n" << cuboid_pose_fixed.rotation() << "\n");
    if (debug_top_grasps_)
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->publishAxis(cuboid_pose_fixed, rviz_visual_tools::SMALL, "cuboid_pose_fixed");
      visual_tools_->publishWireframeCuboid(cuboid_pose_fixed, depth, width, height, rviz_visual_tools::BLUE);
      visual_tools_->trigger();
      ros::Duration(1).sleep();
      // visual_tools_->prompt("flipped Z by rotating around X");
    }
  }

  // If the ideal top grasp X axis is opposite the top pose then we rotate around Z
  double dot_prodX = (cuboid_pose_fixed.rotation() * Eigen::Vector3d::UnitX())
                         .dot(ideal_grasp_tcp.rotation() * Eigen::Vector3d::UnitX());
  if (dot_prodX < 0)
  {
    ROS_DEBUG_NAMED("generateSuctionGrasps", "flipping X");
    cuboid_pose_fixed = cuboid_pose_fixed * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "New cuboid_direction:\n" << cuboid_pose_fixed.rotation() << "\n");
    if (debug_top_grasps_)
    {
      visual_tools_->deleteAllMarkers();
      visual_tools_->publishAxis(cuboid_pose_fixed, rviz_visual_tools::SMALL, "cuboid_pose_fixed");
      visual_tools_->publishWireframeCuboid(cuboid_pose_fixed, depth, width, height, rviz_visual_tools::GREEN);
      visual_tools_->trigger();
      ros::Duration(1).sleep();
      // visual_tools_->prompt("flipped X by rotating around Z");
    }
  }
  // return cuboid_pose_fixed;
}

// X -> Depth
// Y -> Width
// Z -> Height
bool SuctionGraspGenerator::generateSuctionGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                                  double height, const moveit_grasps::SuctionGraspDataPtr& grasp_data,
                                                  std::vector<GraspCandidatePtr>& grasp_candidates)
{
  // Reset the output
  grasp_candidates.clear();

  Eigen::Isometry3d cuboid_pose_fixed(cuboid_pose);
  orientCuboidTowardsIdealTCP(cuboid_pose_fixed, depth, width, height);
  Eigen::Isometry3d cuboid_top_pose = cuboid_pose_fixed * Eigen::Translation3d(0, 0, -height / 2.0);
  ////////////////
  // Create grasp candidate poses.
  ////////////////
  // First add the center point to ensure that it is a candidate

  if (debug_top_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator",
                           "\n\tWidth:\t" << width << "\n\tDepth:\t" << depth << "\n\tHeight\t" << height);
    visual_tools_->publishAxisLabeled(cuboid_top_pose, "cuboid_top_pose", rviz_visual_tools::SMALL);
    double suction_z_range = grasp_data->grasp_max_depth_ - grasp_data->grasp_min_depth_;
    visual_tools_->publishWireframeCuboid(cuboid_top_pose * Eigen::Translation3d(0, 0, suction_z_range / 2.0), depth,
                                          width, suction_z_range, rviz_visual_tools::RED);
    visual_tools_->trigger();
  }

  EigenSTL::vector_Isometry3d grasp_poses_tcp;
  grasp_poses_tcp.emplace_back(cuboid_top_pose * Eigen::Translation3d(0, 0, grasp_data->grasp_min_depth_));

  // We define min, max and inc for each for loop here for readability

  // if X range is less than y range then we use x range for the xy range
  double xy_increment = grasp_data->grasp_resolution_;
  double y_min = 0;
  double y_max = (width + grasp_data->suction_voxel_matrix_->getActiveSuctionWidthY()) / 2.0 - xy_increment;

  double x_min = 0;
  double x_max = (depth + grasp_data->suction_voxel_matrix_->getActiveSuctionWidthX()) / 2.0 - xy_increment;

  double z_increment = grasp_data->grasp_depth_resolution_;
  double z_min = z_increment;
  double z_max = grasp_data->grasp_max_depth_ - grasp_data->grasp_min_depth_;

  double yaw_increment = M_PI * (grasp_data->angle_resolution_ / 180.0);
  double yaw_min = yaw_increment;
  double yaw_max = 2.0 * M_PI;

  // clang-format off
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "x_min:                  " << x_min);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "x_max:                  " << x_max);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "depth:                  " << depth);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "active_suction_range_x: " << grasp_data->suction_voxel_matrix_->getActiveSuctionWidthX());
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "voxel_x_width:          " << grasp_data->suction_voxel_matrix_->getVoxelWidthX());
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "y_min:                  " << y_min);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "y_max:                  " << y_max);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "width:                  " << width);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "active_suction_range_y: " << grasp_data->suction_voxel_matrix_->getActiveSuctionWidthY());
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "voxel_y_width:          " << grasp_data->suction_voxel_matrix_->getVoxelWidthY());
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "z_min:                  " << z_min);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "z_max:                  " << z_max);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "xy_increment:           " << xy_increment);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "yaw_increment:          " << yaw_increment);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "yaw_min:                " << yaw_min);
  ROS_DEBUG_STREAM_NAMED("grasp_generator.suction.range", "yaw_max:                " << yaw_max);

  // clang-format on

  // For each range (X, Y, Z, Yaw) create copies of the grasp poses for each value in the range
  std::size_t num_grasps;

  // Add Depth grasps (Z-axis)
  num_grasps = grasp_poses_tcp.size();
  ROS_DEBUG_STREAM_NAMED("grasp_generator.generate_suction_grasps", "num grasps before Z:\t " << num_grasps);
  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    for (double z = z_min; z <= z_max; z += z_increment)
    {
      grasp_poses_tcp.emplace_back(grasp_poses_tcp[i] * Eigen::Translation3d(0, 0, z));
    }
  }
  num_grasps = grasp_poses_tcp.size();
  ROS_DEBUG_STREAM_NAMED("grasp_generator.generate_suction_grasps", "num grasps after Z:\t " << num_grasps);

  // Add Y translation grasps
  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    for (double y = y_max; y >= y_min; y -= xy_increment)
    {
      grasp_poses_tcp.emplace_back(grasp_poses_tcp[i] * Eigen::Translation3d(0, y, 0));
      grasp_poses_tcp.emplace_back(grasp_poses_tcp[i] * Eigen::Translation3d(0, -y, 0));
    }
  }
  num_grasps = grasp_poses_tcp.size();
  ROS_DEBUG_STREAM_NAMED("grasp_generator.generate_suction_grasps", "num grasps after Y:\t " << num_grasps);

  // Add X translation grasps
  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    for (double x = x_max; x >= x_min; x -= xy_increment)
    {
      grasp_poses_tcp.emplace_back(grasp_poses_tcp[i] * Eigen::Translation3d(x, 0, 0));
      grasp_poses_tcp.emplace_back(grasp_poses_tcp[i] * Eigen::Translation3d(-x, 0, 0));
    }
  }
  num_grasps = grasp_poses_tcp.size();
  ROS_DEBUG_STREAM_NAMED("grasp_generator.generate_suction_grasps", "num grasps after X:\t " << num_grasps);

  // Add rotated suction grasps (Yaw)
  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    for (double yaw = yaw_min; yaw < yaw_max; yaw += yaw_increment)
    {
      grasp_poses_tcp.emplace_back(grasp_poses_tcp[i] * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }
  }
  num_grasps = grasp_poses_tcp.size();
  ROS_DEBUG_STREAM_NAMED("grasp_generator.generate_suction_grasps", "num grasps after Yaw:\t " << num_grasps);

  Eigen::Vector3d object_size(depth, width, height);
  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    Eigen::Isometry3d grasp_pose_eef_mount = grasp_poses_tcp[i] * grasp_data->tcp_to_eef_mount_;
    addGrasp(grasp_pose_eef_mount, grasp_data, cuboid_top_pose, object_size, grasp_candidates);
    if (debug_top_grasps_)
    {
      visual_tools_->publishAxis(grasp_poses_tcp[i], rviz_visual_tools::MEDIUM, "tcp pose");
      visual_tools_->publishAxis(grasp_pose_eef_mount, rviz_visual_tools::SMALL, "eef_mount pose");
    }
  }

  if (debug_top_grasps_)
  {
    Eigen::Isometry3d ideal_copy = ideal_grasp_pose_;
    ideal_copy.translation() += Eigen::Vector3d(0.0, 0.0, 1.0);
    visual_tools_->publishAxisLabeled(ideal_copy, "ideal grasp orientation", rviz_visual_tools::MEDIUM);
    visual_tools_->trigger();
  }

  if (!grasp_candidates.size())
    ROS_WARN_STREAM_NAMED("grasp_generator.generate_suction_grasps", "Generated 0 grasps");
  else
    ROS_INFO_STREAM_NAMED("grasp_generator.generate_suction_grasps", "Generated " << grasp_candidates.size()
                                                                                  << " grasp"
                                                                                     "s");

  // Visualize animated grasps that have been generated
  if (show_prefiltered_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator.generate_suction_grasps", "Animating all generated (candidate) grasps "
                                                                      "before filtering");
    visualizeAnimatedGrasps(grasp_candidates, grasp_data->ee_jmg_, show_prefiltered_grasps_speed_);
  }

  return true;
}

}  // namespace moveit_grasps
