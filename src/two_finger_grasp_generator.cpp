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

#include <moveit_grasps/two_finger_grasp_generator.h>
#include <moveit_grasps/grasp_filter.h>

#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace
{
void debugFailedOpenGripper(double percent_open, double min_finger_open_on_approach, double object_width,
                            double grasp_padding_on_approach)
{
  ROS_ERROR_STREAM_NAMED("grasp_generator", "Unable to set grasp width to "
                                                << percent_open << " % open. Stats:"
                                                << "\n min_finger_open_on_approach: \t " << min_finger_open_on_approach
                                                << "\n object_width: \t " << object_width
                                                << "\n grasp_padding_on_approach_: \t " << grasp_padding_on_approach);
}

}  // namespace

namespace moveit_grasps
{
TwoFingerGraspCandidateConfig::TwoFingerGraspCandidateConfig()
  : enable_corner_grasps_(true)
  , enable_face_grasps_(true)
  , enable_variable_angle_grasps_(true)
  , enable_edge_grasps_(true)
  , generate_x_axis_grasps_(true)
  , generate_y_axis_grasps_(true)
  , generate_z_axis_grasps_(true)
{
}
void TwoFingerGraspCandidateConfig::enableAllGraspTypes()
{
  enable_corner_grasps_ = true;
  enable_face_grasps_ = true;
  enable_variable_angle_grasps_ = true;
  enable_edge_grasps_ = true;
}
void TwoFingerGraspCandidateConfig::enableAllGraspAxes()
{
  generate_x_axis_grasps_ = true;
  generate_y_axis_grasps_ = true;
  generate_z_axis_grasps_ = true;
}
void TwoFingerGraspCandidateConfig::enableAll()
{
  enableAllGraspTypes();
  enableAllGraspAxes();
}
void TwoFingerGraspCandidateConfig::disableAllGraspTypes()
{
  enable_corner_grasps_ = false;
  enable_face_grasps_ = false;
  enable_variable_angle_grasps_ = false;
  enable_edge_grasps_ = false;
}
void TwoFingerGraspCandidateConfig::disableAllGraspAxes()
{
  generate_x_axis_grasps_ = false;
  generate_y_axis_grasps_ = false;
  generate_z_axis_grasps_ = false;
}
void TwoFingerGraspCandidateConfig::disableAll()
{
  disableAllGraspTypes();
  disableAllGraspAxes();
}

// Constructor
TwoFingerGraspGenerator::TwoFingerGraspGenerator(const moveit_visual_tools::MoveItVisualToolsPtr& visual_tools,
                                                 bool verbose)
  : GraspGenerator(visual_tools, verbose)
{
  auto two_finger_grasp_score_weights = std::make_shared<TwoFingerGraspScoreWeights>();
  grasp_score_weights_ = std::dynamic_pointer_cast<GraspScoreWeights>(two_finger_grasp_score_weights);

  grasp_candidate_config_ = TwoFingerGraspCandidateConfig();
}

void TwoFingerGraspGenerator::setGraspCandidateConfig(const TwoFingerGraspCandidateConfig& grasp_candidate_config)
{
  grasp_candidate_config_ = grasp_candidate_config;
}

bool TwoFingerGraspGenerator::generateGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                             double height, const moveit_grasps::GraspDataPtr& grasp_data,
                                             std::vector<GraspCandidatePtr>& grasp_candidates)
{
  auto two_finger_grasp_data = std::dynamic_pointer_cast<TwoFingerGraspData>(grasp_data);
  if (!two_finger_grasp_data)
  {
    ROS_ERROR_STREAM_NAMED("grasp_generator",
                           "grasp_data is not castable to TwoFingerGraspData. Make sure you are using "
                           "the child class");
    return false;
  }
  return generateGrasps(cuboid_pose, depth, width, height, two_finger_grasp_data, grasp_candidates);
}

bool TwoFingerGraspGenerator::generateGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                             double height, const TwoFingerGraspDataPtr& grasp_data,
                                             std::vector<GraspCandidatePtr>& grasp_candidates)
{
  bool result =
      generateFingerGrasps(cuboid_pose, depth, width, height, grasp_data, grasp_candidates, grasp_candidate_config_);

  if (result)
    std::sort(grasp_candidates.begin(), grasp_candidates.end(), GraspFilter::compareGraspScores);

  return result;
}

bool TwoFingerGraspGenerator::addGrasp(const Eigen::Isometry3d& grasp_pose_eef_mount,
                                       const TwoFingerGraspDataPtr& grasp_data, const Eigen::Isometry3d& object_pose,
                                       const Eigen::Vector3d& object_size, double object_width,
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

  // set minimum opening of fingers for pre grasp approach
  double min_finger_open_on_approach = object_width + 2 * grasp_data->grasp_padding_on_approach_;
  double percent_open;

  // Create grasp with widest fingers possible ----------------------------------------------
  percent_open = 1.0;
  if (!grasp_data->setGraspWidth(percent_open, min_finger_open_on_approach, new_grasp.pre_grasp_posture))
  {
    debugFailedOpenGripper(percent_open, min_finger_open_on_approach, object_width,
                           grasp_data->grasp_padding_on_approach_);
    return false;
  }

  new_grasp.grasp_quality = scoreFingerGrasp(grasp_pose_tcp, grasp_data, object_pose, percent_open);

  // Show visualization for widest grasp

  grasp_candidates.push_back(std::make_shared<GraspCandidate>(new_grasp, grasp_data, object_pose));

  // Create grasp with middle width fingers -------------------------------------------------
  percent_open = 0.5;
  if (!grasp_data->setGraspWidth(percent_open, min_finger_open_on_approach, new_grasp.pre_grasp_posture))
  {
    debugFailedOpenGripper(percent_open, min_finger_open_on_approach, object_width,
                           grasp_data->grasp_padding_on_approach_);
    return false;
  }
  new_grasp.grasp_quality = scoreFingerGrasp(grasp_pose_tcp, grasp_data, object_pose, percent_open);
  grasp_candidates.push_back(std::make_shared<GraspCandidate>(new_grasp, grasp_data, object_pose));

  // Create grasp with fingers at minimum width ---------------------------------------------
  percent_open = 0.0;
  if (!grasp_data->setGraspWidth(percent_open, min_finger_open_on_approach, new_grasp.pre_grasp_posture))
  {
    debugFailedOpenGripper(percent_open, min_finger_open_on_approach, object_width,
                           grasp_data->grasp_padding_on_approach_);
    return false;
  }
  new_grasp.grasp_quality = scoreFingerGrasp(grasp_pose_tcp, grasp_data, object_pose, percent_open);
  grasp_candidates.push_back(std::make_shared<GraspCandidate>(new_grasp, grasp_data, object_pose));

  return true;
}

bool TwoFingerGraspGenerator::generateCuboidAxisGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                                       double height, grasp_axis_t axis,
                                                       const TwoFingerGraspDataPtr& grasp_data,
                                                       const TwoFingerGraspCandidateConfig& grasp_candidate_config,
                                                       std::vector<GraspCandidatePtr>& grasp_candidates)
{
  double finger_depth = grasp_data->grasp_max_depth_ - grasp_data->grasp_min_depth_;
  double length_along_a, length_along_b, length_along_c;
  double delta_a, delta_b, delta_f;
  double alpha_x, alpha_y, alpha_z;
  Eigen::Vector3d object_size(depth, width, height);

  double object_width;
  EigenSTL::vector_Isometry3d grasp_poses_tcp;

  Eigen::Isometry3d grasp_pose_tcp = cuboid_pose;
  Eigen::Vector3d a_dir, b_dir, c_dir;

  if (axis == X_AXIS)
  {
    length_along_a = width;
    length_along_b = height;
    length_along_c = depth;
    a_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitY();
    b_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitZ();
    c_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitX();
    alpha_x = -M_PI / 2.0;
    alpha_y = 0;
    alpha_z = -M_PI / 2.0;
    object_width = depth;
  }
  else if (axis == Y_AXIS)
  {
    length_along_a = depth;
    length_along_b = height;
    length_along_c = width;
    a_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitX();
    b_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitZ();
    c_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitY();
    alpha_x = 0;
    alpha_y = M_PI / 2.0;
    alpha_z = M_PI;
    object_width = width;
  }
  else  // Z_AXIS
  {
    length_along_a = depth;
    length_along_b = width;
    length_along_c = height;
    a_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitX();
    b_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitY();
    c_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitZ();
    alpha_x = M_PI / 2.0;
    alpha_y = M_PI / 2.0;
    alpha_z = 0;
    object_width = height;
  }

  double rotation_angles[3];
  rotation_angles[0] = alpha_x;
  rotation_angles[1] = alpha_y;
  rotation_angles[2] = alpha_z;

  a_dir = a_dir.normalized();
  b_dir = b_dir.normalized();
  c_dir = c_dir.normalized();

  // Add grasps at corners, grasps are centroid aligned
  double offset = 0.001;  // back the palm off of the object slightly
  Eigen::Vector3d corner_translation_a;
  Eigen::Vector3d corner_translation_b;
  double angle_res = grasp_data->angle_resolution_ * M_PI / 180.0;
  std::size_t num_radial_grasps = ceil((M_PI / 2.0) / angle_res);
  Eigen::Vector3d translation;

  if (grasp_candidate_config.enable_corner_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "adding corner grasps...");
    corner_translation_a = 0.5 * (length_along_a + offset) * a_dir;
    corner_translation_b = 0.5 * (length_along_b + offset) * b_dir;

    if (num_radial_grasps <= 0)
      num_radial_grasps = 1;

    // move to corner 0.5 * ( -a, -b)
    translation = -corner_translation_a - corner_translation_b;
    addCornerGraspsHelper(cuboid_pose, rotation_angles, translation, 0.0, num_radial_grasps, grasp_poses_tcp);

    // move to corner 0.5 * ( -a, +b)
    translation = -corner_translation_a + corner_translation_b;
    addCornerGraspsHelper(cuboid_pose, rotation_angles, translation, -M_PI / 2.0, num_radial_grasps, grasp_poses_tcp);

    // move to corner 0.5 * ( +a, +b)
    translation = corner_translation_a + corner_translation_b;
    addCornerGraspsHelper(cuboid_pose, rotation_angles, translation, M_PI, num_radial_grasps, grasp_poses_tcp);

    // move to corner 0.5 * ( +a, -b)
    translation = corner_translation_a - corner_translation_b;
    addCornerGraspsHelper(cuboid_pose, rotation_angles, translation, M_PI / 2.0, num_radial_grasps, grasp_poses_tcp);
  }
  std::size_t num_corner_grasps = grasp_poses_tcp.size();

  // Create grasps along faces of cuboid, grasps are axis aligned
  std::size_t num_grasps_along_a;
  std::size_t num_grasps_along_b;
  double rotation;
  Eigen::Vector3d a_translation;
  Eigen::Vector3d b_translation;
  Eigen::Vector3d delta;

  // get exact deltas for sides from desired delta
  num_grasps_along_a = floor((length_along_a - grasp_data->gripper_finger_width_) / grasp_data->grasp_resolution_) + 1;
  num_grasps_along_b = floor((length_along_b - grasp_data->gripper_finger_width_) / grasp_data->grasp_resolution_) + 1;

  // if the gripper fingers are wider than the object we're trying to grasp, try with gripper aligned with
  // top/center/bottom of object
  // note that current implementation limits objects that are the same size as the gripper_finger_width to 1 grasp
  if (num_grasps_along_a <= 0)
  {
    delta_a = length_along_a - grasp_data->gripper_finger_width_ / 2.0;
    num_grasps_along_a = 3;
  }
  if (num_grasps_along_b <= 0)
  {
    delta_b = length_along_b - grasp_data->gripper_finger_width_ / 2.0;
    num_grasps_along_b = 3;
  }

  if (num_grasps_along_a == 1)
    delta_a = 0;
  else
    delta_a = (length_along_a - grasp_data->gripper_finger_width_) / static_cast<double>(num_grasps_along_a - 1);

  if (num_grasps_along_b == 1)
    delta_b = 0;
  else
    delta_b = (length_along_b - grasp_data->gripper_finger_width_) / static_cast<double>(num_grasps_along_b - 1);

  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "delta_a : delta_b = " << delta_a << " : " << delta_b);
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "num_grasps_along_a : num_grasps_along_b  = "
                                                   << num_grasps_along_a << " : " << num_grasps_along_b);

  // TODO(mlautman): There is a bug with face grasps allowing the grasp generator to generate grasps where the gripper
  // fingers
  //                 are in collision with the object being grasped
  if (grasp_candidate_config.enable_face_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "adding face grasps...");

    a_translation = -(0.5 * (length_along_a + offset) * a_dir) -
                    0.5 * (length_along_b - grasp_data->gripper_finger_width_) * b_dir - delta_b * b_dir;
    b_translation = -0.5 * (length_along_a - grasp_data->gripper_finger_width_) * a_dir - delta_a * a_dir -
                    (0.5 * (length_along_b + offset) * b_dir);

    // grasps along -a_dir face
    delta = delta_b * b_dir;
    rotation = 0.0;
    addFaceGraspsHelper(cuboid_pose, rotation_angles, a_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp);

    // grasps along +b_dir face
    rotation = -M_PI / 2.0;
    delta = -delta_a * a_dir;
    addFaceGraspsHelper(cuboid_pose, rotation_angles, -b_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp);

    // grasps along +a_dir face
    rotation = M_PI;
    delta = -delta_b * b_dir;
    addFaceGraspsHelper(cuboid_pose, rotation_angles, -a_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp);

    // grasps along -b_dir face
    rotation = M_PI / 2.0;
    delta = delta_a * a_dir;
    addFaceGraspsHelper(cuboid_pose, rotation_angles, b_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp);
  }

  // add grasps at variable angles
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "adding variable angle grasps...");
  Eigen::Isometry3d base_pose;
  std::size_t num_grasps = grasp_poses_tcp.size();
  if (grasp_candidate_config.enable_variable_angle_grasps_)
  {
    // corner grasps at zero depth don't need variable angles
    for (std::size_t i = num_corner_grasps; i < num_grasps; ++i)
    {
      base_pose = grasp_poses_tcp[i];

      grasp_pose_tcp = base_pose * Eigen::AngleAxisd(angle_res, Eigen::Vector3d::UnitY());
      std::size_t max_iterations = M_PI / angle_res + 1;
      std::size_t iterations = 0;
      while (graspIntersectionHelper(cuboid_pose, depth, width, height, grasp_pose_tcp, grasp_data))
      {
        grasp_poses_tcp.push_back(grasp_pose_tcp);
        grasp_pose_tcp *= Eigen::AngleAxisd(angle_res, Eigen::Vector3d::UnitY());
        iterations++;
        if (iterations > max_iterations)
        {
          ROS_WARN_STREAM_NAMED("cuboid_axis_grasps", "exceeded max iterations while creating variable angle grasps");
          break;
        }
      }

      iterations = 0;
      grasp_pose_tcp = base_pose * Eigen::AngleAxisd(-angle_res, Eigen::Vector3d::UnitY());
      while (graspIntersectionHelper(cuboid_pose, depth, width, height, grasp_pose_tcp, grasp_data))
      {
        grasp_poses_tcp.push_back(grasp_pose_tcp);
        // visual_tools_->publishZArrow(grasp_pose_tcp, rviz_visual_tools::CYAN, rviz_visual_tools::XSMALL, 0.02);
        grasp_pose_tcp *= Eigen::AngleAxisd(-angle_res, Eigen::Vector3d::UnitY());
        // ros::Duration(0.2).sleep();
        iterations++;
        if (iterations > max_iterations)
        {
          ROS_WARN_STREAM_NAMED("cuboid_axis_grasps", "exceeded max iterations while creating variable angle grasps");
          break;
        }
      }
    }
  }

  if (grasp_candidate_config.enable_edge_grasps_)
  {
    // Add grasps along edges
    // move grasp pose to edge of cuboid
    double a_sign = 1.0;
    double b_sign = 1.0;
    double a_rot_sign = 1.0;
    double b_rot_sign = 1.0;

    if (axis == Y_AXIS)
    {
      a_sign = -1.0;
      b_rot_sign = -1.0;
    }

    if (axis == Z_AXIS)
    {
      a_sign = -1.0;
      b_sign = -1.0;
      a_rot_sign = -1.0;
      b_rot_sign = -1.0;
    }

    a_translation = -0.5 * (length_along_a + offset) * a_dir -
                    0.5 * (length_along_b - grasp_data->gripper_finger_width_) * b_dir - delta_b * b_dir -
                    0.5 * (length_along_c + offset) * c_dir * a_sign;
    b_translation = -0.5 * (length_along_a - grasp_data->gripper_finger_width_) * a_dir - delta_a * a_dir -
                    (0.5 * (length_along_b + offset) * b_dir) - 0.5 * (length_along_c + offset) * c_dir * b_sign;

    // grasps along -a_dir face
    delta = delta_b * b_dir;
    rotation = 0.0;
    addEdgeGraspsHelper(cuboid_pose, rotation_angles, a_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp, -M_PI / 4.0 * a_rot_sign);

    // grasps along +b_dir face
    rotation = -M_PI / 2.0;
    delta = -delta_a * a_dir;
    addEdgeGraspsHelper(cuboid_pose, rotation_angles, -b_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp, M_PI / 4.0 * b_rot_sign);

    // grasps along +a_dir face
    rotation = M_PI;
    delta = -delta_b * b_dir;
    addEdgeGraspsHelper(cuboid_pose, rotation_angles, -a_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp, M_PI / 4.0 * a_rot_sign);

    // grasps along -b_dir face
    rotation = M_PI / 2.0;
    delta = delta_a * a_dir;
    addEdgeGraspsHelper(cuboid_pose, rotation_angles, b_translation, delta, rotation, num_grasps_along_b,
                        grasp_poses_tcp, -M_PI / 4.0 * b_rot_sign);
  }
  // Add grasps at variable depths
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "adding depth grasps...");
  std::size_t num_depth_grasps = ceil(finger_depth / grasp_data->grasp_depth_resolution_);
  if (num_depth_grasps <= 0)
    num_depth_grasps = 1;
  delta_f = finger_depth / static_cast<double>(num_depth_grasps);

  num_grasps = grasp_poses_tcp.size();
  Eigen::Vector3d grasp_dir;
  Eigen::Isometry3d depth_pose;

  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    grasp_dir = grasp_poses_tcp[i].rotation() * Eigen::Vector3d::UnitZ();
    depth_pose = grasp_poses_tcp[i];
    for (std::size_t j = 0; j < num_depth_grasps; ++j)
    {
      depth_pose.translation() += delta_f * grasp_dir;
      grasp_poses_tcp.push_back(depth_pose);
    }
  }

  // add grasps in both directions
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "adding bi-directional grasps...");
  num_grasps = grasp_poses_tcp.size();
  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    grasp_pose_tcp = grasp_poses_tcp[i];
    grasp_pose_tcp *= Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitZ());
    grasp_poses_tcp.push_back(grasp_pose_tcp);
  }

  // compute min/max distances to object
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps", "computing min/max grasp distance...");
  num_grasps = grasp_poses_tcp.size();
  min_grasp_distance_ = std::numeric_limits<double>::max();
  max_grasp_distance_ = std::numeric_limits<double>::min();
  min_translations_ = Eigen::Vector3d(std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),
                                      std::numeric_limits<double>::max());
  max_translations_ = Eigen::Vector3d(std::numeric_limits<double>::min(), std::numeric_limits<double>::min(),
                                      std::numeric_limits<double>::min());
  double grasp_distance;

  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    grasp_pose_tcp = grasp_poses_tcp[i];
    grasp_distance = (grasp_pose_tcp.translation() - cuboid_pose.translation()).norm();
    if (grasp_distance > max_grasp_distance_)
      max_grasp_distance_ = grasp_distance;

    if (grasp_distance < min_grasp_distance_)
      min_grasp_distance_ = grasp_distance;

    for (std::size_t j = 0; j < 3; ++j)
    {
      if (grasp_pose_tcp.translation()[j] < min_translations_[j])
        min_translations_[j] = grasp_pose_tcp.translation()[j];

      if (grasp_pose_tcp.translation()[j] > max_translations_[j])
        max_translations_[j] = grasp_pose_tcp.translation()[j];
    }
  }

  ROS_DEBUG_STREAM_NAMED("grasp_generator.add",
                         "min/max distance = " << min_grasp_distance_ << ", " << max_grasp_distance_);

  // add all poses as possible grasps
  std::size_t num_grasps_added = 0;

  for (std::size_t i = 0; i < grasp_poses_tcp.size(); ++i)
  {
    Eigen::Isometry3d grasp_pose_eef_mount = grasp_poses_tcp[i] * grasp_data->tcp_to_eef_mount_;
    if (!addGrasp(grasp_pose_eef_mount, grasp_data, cuboid_pose, object_size, object_width, grasp_candidates))
    {
      ROS_DEBUG_STREAM_NAMED("grasp_generator.add", "Unable to add grasp - function returned false");
    }
    else
      num_grasps_added++;
  }
  ROS_INFO_STREAM_NAMED("grasp_generator.add", "\033[1;36madded " << num_grasps_added << " of "
                                                                  << grasp_poses_tcp.size()
                                                                  << " grasp poses created\033[0m");
  return true;
}

std::size_t TwoFingerGraspGenerator::addCornerGraspsHelper(const Eigen::Isometry3d& pose, double rotation_angles[3],
                                                           const Eigen::Vector3d& translation, double corner_rotation,
                                                           std::size_t num_radial_grasps,
                                                           EigenSTL::vector_Isometry3d& grasp_poses_tcp)
{
  std::size_t num_grasps_added = 0;
  double delta_angle = (M_PI / 2.0) / static_cast<double>(num_radial_grasps + 1);
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "delta_angle = " << delta_angle);
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "num_radial_grasps = " << num_radial_grasps);

  // rotate & translate pose to be aligned with edge of cuboid
  Eigen::Isometry3d grasp_pose_tcp = pose;
  grasp_pose_tcp *= Eigen::AngleAxisd(rotation_angles[0], Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(rotation_angles[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rotation_angles[2], Eigen::Vector3d::UnitZ());
  grasp_pose_tcp *= Eigen::AngleAxisd(corner_rotation, Eigen::Vector3d::UnitY());
  grasp_pose_tcp.translation() += translation;

  for (std::size_t i = 0; i < num_radial_grasps; ++i)
  {
    // Eigen::Vector3d grasp_dir = grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitZ();
    // Eigen::Isometry3d radial_pose = grasp_pose_tcp;
    grasp_pose_tcp *= Eigen::AngleAxisd(delta_angle, Eigen::Vector3d::UnitY());
    grasp_poses_tcp.push_back(grasp_pose_tcp);
    num_grasps_added++;
  }
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "num_grasps_added : grasp_poses_tcp.size() = "
                                                          << num_grasps_added << " : " << grasp_poses_tcp.size());
  return num_grasps_added;
}

std::size_t TwoFingerGraspGenerator::addFaceGraspsHelper(const Eigen::Isometry3d& pose, double rotation_angles[3],
                                                         const Eigen::Vector3d& translation,
                                                         const Eigen::Vector3d& delta, double alignment_rotation,
                                                         std::size_t num_grasps,
                                                         EigenSTL::vector_Isometry3d& grasp_poses_tcp)
{
  std::size_t num_grasps_added = 0;
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "delta = \n" << delta);
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "num_grasps = " << num_grasps);

  Eigen::Isometry3d grasp_pose_tcp = pose;
  grasp_pose_tcp *= Eigen::AngleAxisd(rotation_angles[0], Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(rotation_angles[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rotation_angles[2], Eigen::Vector3d::UnitZ());
  grasp_pose_tcp *= Eigen::AngleAxisd(alignment_rotation, Eigen::Vector3d::UnitY());
  grasp_pose_tcp.translation() += translation;

  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    grasp_pose_tcp.translation() += delta;
    grasp_poses_tcp.push_back(grasp_pose_tcp);
    num_grasps_added++;
  }
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "num_grasps_added : grasp_poses_tcp.size() = "
                                                          << num_grasps_added << " : " << grasp_poses_tcp.size());
  return true;
}

std::size_t TwoFingerGraspGenerator::addEdgeGraspsHelper(const Eigen::Isometry3d& pose, double rotation_angles[3],
                                                         const Eigen::Vector3d& translation,
                                                         const Eigen::Vector3d& delta, double alignment_rotation,
                                                         std::size_t num_grasps,
                                                         EigenSTL::vector_Isometry3d& grasp_poses_tcp,
                                                         double corner_rotation)
{
  std::size_t num_grasps_added = 0;
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "delta = \n" << delta);
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "num_grasps = " << num_grasps);

  Eigen::Isometry3d grasp_pose_tcp = pose;
  grasp_pose_tcp *= Eigen::AngleAxisd(rotation_angles[0], Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(rotation_angles[1], Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(rotation_angles[2], Eigen::Vector3d::UnitZ());
  grasp_pose_tcp *= Eigen::AngleAxisd(alignment_rotation, Eigen::Vector3d::UnitY());

  // rotate towards cuboid
  grasp_pose_tcp *= Eigen::AngleAxisd(corner_rotation, Eigen::Vector3d::UnitX());
  grasp_pose_tcp.translation() += translation;

  for (std::size_t i = 0; i < num_grasps; ++i)
  {
    grasp_pose_tcp.translation() += delta;
    grasp_poses_tcp.push_back(grasp_pose_tcp);
    num_grasps_added++;
  }
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps.helper", "num_grasps_added : grasp_poses_tcp.size() = "
                                                          << num_grasps_added << " : " << grasp_poses_tcp.size());
  return true;
}

bool TwoFingerGraspGenerator::graspIntersectionHelper(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                                      double height, const Eigen::Isometry3d& grasp_pose_tcp,
                                                      const TwoFingerGraspDataPtr& grasp_data)
{
  // TODO(davetcoleman): add parameter to enable vizualization commented lines after further testing

  // get line segment from grasp point to fingertip
  Eigen::Vector3d point_a = grasp_pose_tcp.translation();
  Eigen::Vector3d point_b =
      point_a + grasp_pose_tcp.rotation() * Eigen::Vector3d::UnitZ() * grasp_data->grasp_max_depth_;

  // translate points into cuboid coordinate system
  point_a = cuboid_pose.inverse() * point_a;  // T_cuboid-world * p_world = p_cuboid
  point_b = cuboid_pose.inverse() * point_b;

  double t, u, v;
  Eigen::Vector3d intersection;
  // check if line segment intersects XY faces of cuboid (z = +/- height/2)
  t = (height / 2.0 - point_a[2]) / (point_b[2] - point_a[2]);  // parameterization of line segment in 3d
  if (intersectionHelper(t, point_a[0], point_a[1], point_b[0], point_b[1], depth, width, u, v))
  {
    return true;
  }

  t = (-height / 2.0 - point_a[2]) / (point_b[2] - point_a[2]);
  if (intersectionHelper(t, point_a[0], point_a[1], point_b[0], point_b[1], depth, width, u, v))
  {
    return true;
  }

  // check if line segment intersects XZ faces of cuboid (y = +/- width/2)
  t = (width / 2.0 - point_a[1]) / (point_b[1] - point_a[1]);
  if (intersectionHelper(t, point_a[0], point_a[2], point_b[0], point_b[2], depth, height, u, v))
  {
    return true;
  }

  t = (-width / 2.0 - point_a[1]) / (point_b[1] - point_a[1]);
  if (intersectionHelper(t, point_a[0], point_a[2], point_b[0], point_b[2], depth, height, u, v))
  {
    return true;
  }

  // check if line segment intersects YZ faces of cuboid (x = +/- depth/2)
  t = (depth / 2.0 - point_a[0]) / (point_b[0] - point_a[0]);
  if (intersectionHelper(t, point_a[1], point_a[2], point_b[1], point_b[2], width, height, u, v))
  {
    return true;
  }

  t = (-depth / 2.0 - point_a[0]) / (point_b[0] - point_a[0]);
  if (intersectionHelper(t, point_a[1], point_a[2], point_b[1], point_b[2], width, height, u, v))
  {
    return true;
  }

  // no intersection found
  return false;
}

bool TwoFingerGraspGenerator::intersectionHelper(double t, double u1, double v1, double u2, double v2, double a,
                                                 double b, double& u, double& v)
{
  // plane must cross through our line segment
  if (t >= 0 && t <= 1)
  {
    u = u1 + t * (u2 - u1);
    v = v1 + t * (v2 - v1);

    if (u >= -a / 2 && u <= a / 2 && v >= -b / 2 && v <= b / 2)
      return true;
  }

  return false;
}

double TwoFingerGraspGenerator::scoreFingerGrasp(const Eigen::Isometry3d& grasp_pose_tcp,
                                                 const TwoFingerGraspDataPtr& grasp_data,
                                                 const Eigen::Isometry3d& object_pose, double percent_open)
{
  static const std::string logger_name = "grasp_generator.scoreGrasp";
  ROS_DEBUG_STREAM_NAMED(logger_name, "starting to score grasp...");

  // get portion of score based on the gripper's opening width on approach
  double width_score = TwoFingerGraspScorer::scoreGraspWidth(grasp_data, percent_open);

  // get portion of score based on the pinchers being down
  Eigen::Vector3d orientation_scores =
      TwoFingerGraspScorer::scoreRotationsFromDesired(grasp_pose_tcp, ideal_grasp_pose_);

  // get portion of score based on the distance of the grasp pose to the object pose

  // NOTE: when this function is called we've lost the references to the acutal size of the object.
  // max_distance should be the length of the fingers minus some minimum amount that the fingers need to grip an object
  // since we don't know the distance from the centoid of the object to the edge of the object, this is set as an
  // arbitrary number given our target object set
  double distance_score = TwoFingerGraspScorer::scoreDistanceToPalm(grasp_pose_tcp, grasp_data, object_pose,
                                                                    min_grasp_distance_, max_grasp_distance_);

  // should really change this to be like orienation_scores so we can score any translation
  Eigen::Vector3d translation_scores =
      TwoFingerGraspScorer::scoreGraspTranslation(grasp_pose_tcp, min_translations_, max_translations_);

  // want minimum translation
  translation_scores *= -1.0;
  translation_scores += Eigen::Vector3d(1.0, 1.0, 1.0);

  double total_score = 0;
  auto two_finger_grasp_score_weights = std::dynamic_pointer_cast<TwoFingerGraspScoreWeights>(grasp_score_weights_);
  if (two_finger_grasp_score_weights)
  {
    total_score = two_finger_grasp_score_weights->computeScore(orientation_scores, translation_scores, distance_score,
                                                               width_score, getVerbose());
  }
  else
  {
    ROS_WARN_NAMED(logger_name, "Failed to cast grasp_score_weights_ as TwoFingerGraspScoreWeights. continuing without "
                                "finger specific scores");
    total_score = grasp_score_weights_->computeScore(orientation_scores, translation_scores, getVerbose());
  }

  return total_score;
}

bool TwoFingerGraspGenerator::generateFingerGrasps(const Eigen::Isometry3d& cuboid_pose, double depth, double width,
                                                   double height, const TwoFingerGraspDataPtr& grasp_data,
                                                   std::vector<GraspCandidatePtr>& grasp_candidates,
                                                   const TwoFingerGraspCandidateConfig& grasp_candidate_config)
{
  // Generate grasps over axes that aren't too wide to grip
  // Most default type of grasp is X axis
  TwoFingerGraspCandidateConfig grasp_candidate_config_copy(grasp_candidate_config);
  if (grasp_candidate_config_copy.generate_x_axis_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "Generating grasps around x-axis of cuboid");
    if (depth > grasp_data->max_grasp_width_)  // depth = size along x-axis
    {
      grasp_candidate_config_copy.disableAllGraspTypes();
      grasp_candidate_config_copy.enable_edge_grasps_ = grasp_candidate_config.enable_edge_grasps_;
      grasp_candidate_config_copy.enable_corner_grasps_ = grasp_candidate_config.enable_corner_grasps_;
    }
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, X_AXIS, grasp_data, grasp_candidate_config_copy,
                             grasp_candidates);
  }

  grasp_candidate_config_copy = grasp_candidate_config;
  if (grasp_candidate_config_copy.generate_y_axis_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "Generating grasps around y-axis of cuboid");
    if (width > grasp_data->max_grasp_width_)  // width = size along y-axis
    {
      grasp_candidate_config_copy.disableAllGraspTypes();
      grasp_candidate_config_copy.enable_edge_grasps_ = grasp_candidate_config.enable_edge_grasps_;
      grasp_candidate_config_copy.enable_corner_grasps_ = grasp_candidate_config.enable_corner_grasps_;
    }
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Y_AXIS, grasp_data, grasp_candidate_config_copy,
                             grasp_candidates);
  }

  grasp_candidate_config_copy = grasp_candidate_config;
  if (grasp_candidate_config_copy.generate_z_axis_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "Generating grasps around z-axis of cuboid");
    if (height > grasp_data->max_grasp_width_)  // height = size along z-axis
    {
      grasp_candidate_config_copy.disableAllGraspTypes();
      grasp_candidate_config_copy.enable_edge_grasps_ = grasp_candidate_config.enable_edge_grasps_;
      grasp_candidate_config_copy.enable_corner_grasps_ = grasp_candidate_config.enable_corner_grasps_;
    }
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Z_AXIS, grasp_data, grasp_candidate_config_copy,
                             grasp_candidates);
  }

  if (!grasp_candidates.size())
    ROS_WARN_STREAM_NAMED("grasp_generator", "Generated 0 grasps");
  else
    ROS_INFO_STREAM_NAMED("grasp_generator", "Generated " << grasp_candidates.size() << " grasps");

  // Visualize animated grasps that have been generated
  if (show_prefiltered_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator", "Animating all generated (candidate) grasps before filtering");
    visualizeAnimatedGrasps(grasp_candidates, grasp_data->ee_jmg_, show_prefiltered_grasps_speed_);
  }

  return true;
}

}  // namespace moveit_grasps
