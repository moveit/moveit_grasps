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

/* Author: Dave Coleman <dave@dav.ee>, Andy McEvoy
   Desc:   Generates geometric grasps for cuboids and blocks, not using physics or contact wrenches
*/

#include <moveit_grasps/grasp_generator.h>

// Parameter loading
#include <rviz_visual_tools/ros_param_utilities.h>

namespace moveit_grasps
{

// Constructor
GraspGenerator::GraspGenerator(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose)
  : visual_tools_(visual_tools)
  , nh_("~/generator")
  , verbose_(verbose)
{
  
  mm_between_grasps_ = 0;

  // Load visulization settings
  const std::string parent_name = "grasps"; // for namespacing logging messages
  rviz_visual_tools::getBoolParameter(parent_name, nh_, "verbose", verbose_);
  rviz_visual_tools::getBoolParameter(parent_name, nh_, "show_prefiltered_grasps", show_prefiltered_grasps_);
  rviz_visual_tools::getDoubleParameter(parent_name, nh_, "show_prefiltered_grasps_speed", show_prefiltered_grasps_speed_);

  ROS_DEBUG_STREAM_NAMED("grasps","Loaded grasp generator");
}

bool GraspGenerator::addVariableDepthGrasps(const Eigen::Affine3d& cuboid_pose, const moveit_grasps::GraspDataPtr grasp_data,
                              std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  if (possible_grasps.size() == 0 )
  {
    ROS_WARN_STREAM_NAMED("depth_grasps", "possible_grasps is empty. Call generateGrasps() first");
    return false;
  }

  std::vector<moveit_msgs::Grasp> depth_grasps;
  std::vector<moveit_msgs::Grasp>::iterator it;
  Eigen::Affine3d base_pose, depth_pose;
  Eigen::Vector3d to_cuboid;

  static int grasp_id = 0;
  moveit_msgs::Grasp new_grasp;
  geometry_msgs::PoseStamped depth_pose_msg;
  depth_pose_msg.header.stamp = ros::Time::now();
  depth_pose_msg.header.frame_id = grasp_data->base_link_;

  int number_depth_grasps = grasp_data->finger_to_palm_depth_ / mm_between_grasps_;
  ROS_DEBUG_STREAM_NAMED("depth_grasps","number_depth_grasps = " << number_depth_grasps);
  if (number_depth_grasps < 1)
    number_depth_grasps = 1;
  
  double delta = grasp_data->finger_to_palm_depth_ / number_depth_grasps;

  for (it = possible_grasps.begin(); it != possible_grasps.end(); ++it)
  {
    base_pose = visual_tools_->convertPose(it->grasp_pose.pose);
    to_cuboid = cuboid_pose.translation() - base_pose.translation();
    to_cuboid.normalized();
    to_cuboid *= delta;

    depth_pose = base_pose;
    for (int i = 0; i < 5; i++)
    {
      depth_pose.translation() += to_cuboid;

      new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
      grasp_id++;
      new_grasp.pre_grasp_posture = it->pre_grasp_posture;
      new_grasp.grasp_posture = it->grasp_posture;

      tf::poseEigenToMsg(depth_pose, depth_pose_msg.pose);
      new_grasp.grasp_pose = depth_pose_msg;
      depth_grasps.push_back(new_grasp);

      if (verbose_)
      {
        // show gripper center and grasp direction
        //visual_tools_->publishXArrow(new_grasp.grasp_pose.pose, rviz_visual_tools::RED, rviz_visual_tools::SMALL, 0.05);
        //visual_tools_->publishZArrow(new_grasp.grasp_pose.pose, rviz_visual_tools::BLUE, rviz_visual_tools::SMALL, 0.05);
        visual_tools_->publishBlock(new_grasp.grasp_pose.pose, rviz_visual_tools::PINK, 0.01);

        // Send markers to Rviz
        visual_tools_->triggerBatchPublishAndDisable();
        ros::Duration(0.05).sleep();
    }
    }
  }
  ROS_INFO_STREAM_NAMED("depth_grasps","added " << depth_grasps.size() << " variable depth grasps");
 
  // depth_grasps is almost always larger, should change this so smaller is being inserted.
  possible_grasps.insert(possible_grasps.end(), depth_grasps.begin(), depth_grasps.end());
  return true;
}

bool GraspGenerator::addParallelGrasps(const Eigen::Affine3d& cuboid_pose, 
                                       moveit_grasps::grasp_parallel_plane plane, Eigen::Vector3d grasp_axis,
                                       const moveit_grasps::GraspDataPtr grasp_data,
                                       std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  if (possible_grasps.size() == 0 )
  {
    ROS_WARN_STREAM_NAMED("parallel grasps", "possible_grasps is empty. Call generateGrasps() first");
    return false;
  }

  std::vector<moveit_msgs::Grasp> parallel_grasps;
  std::vector<moveit_msgs::Grasp>::iterator it;
  
  static int grasp_id = 0;
  moveit_msgs::Grasp new_grasp;
  geometry_msgs::PoseStamped parallel_pose_msg;
  parallel_pose_msg.header.stamp = ros::Time::now();
  parallel_pose_msg.header.frame_id = grasp_data->base_link_;


  for (it = possible_grasps.begin(); it != possible_grasps.end(); ++it)
  {
    Eigen::Affine3d parallel_pose = visual_tools_->convertPose(it->grasp_pose.pose);
    Eigen::Vector3d rotation_axis;

    // get angle between grasp
    Eigen::Vector3d parallel_vector;
    parallel_vector = parallel_pose * grasp_axis;

    switch(plane)
    {
      case XY:
        parallel_vector(2) = 0;
        rotation_axis = Eigen::Vector3d::UnitZ();
        break;
      case XZ:
        parallel_vector(1) = 0;
        rotation_axis = Eigen::Vector3d::UnitY();
        break;
      case YZ:
        parallel_vector(0) = 0;
        rotation_axis = Eigen::Vector3d::UnitX();
        break;
      default:
        ROS_WARN_STREAM_NAMED("parallel grasps", "parallel plane not set correctly");
        break;
    }
    
    Eigen::Vector3d to_cuboid = cuboid_pose.translation() - parallel_pose.translation(); 
    Eigen::Vector3d cross_prod = parallel_vector.normalized().cross(to_cuboid.normalized());
    double rotation_angle = acos( to_cuboid.normalized().dot( parallel_vector.normalized() ) );

    // get direction to rotate
    double dot = cross_prod.dot(rotation_axis);
    if (dot < 0)
      rotation_angle *= -1;

    ROS_DEBUG_STREAM_NAMED("parallel grasps", "cross_prod = \n" << cross_prod);
    
    // add new pose
    parallel_pose *= Eigen::AngleAxisd(rotation_angle, rotation_axis);

    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    grasp_id++;
    new_grasp.pre_grasp_posture = it->pre_grasp_posture;
    new_grasp.grasp_posture = it->grasp_posture;
    
    tf::poseEigenToMsg(parallel_pose, parallel_pose_msg.pose);
    new_grasp.grasp_pose = parallel_pose_msg;
    parallel_grasps.push_back(new_grasp);
  }
  
  // should change this so smaller is being inserted.
  possible_grasps.insert(possible_grasps.end(), parallel_grasps.begin(), parallel_grasps.end());
  return true; 
}

bool GraspGenerator::generateGrasps(const shape_msgs::Mesh& mesh_msg, const Eigen::Affine3d& cuboid_pose,
                                    double max_grasp_size, const moveit_grasps::GraspDataPtr grasp_data,
                                    std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  double depth;
  double width;
  double height;
  Eigen::Affine3d mesh_pose;
  if (!getBoundingBoxFromMesh(mesh_msg, mesh_pose, depth, width, height))
  {
    ROS_ERROR_STREAM_NAMED("grasp_generator","Unable to get bounding box from mesh");
    return false;
  }

  // TODO - reconcile the new mesh_pose with the input cuboid_pose

  return generateGrasps(cuboid_pose, depth, width, height, max_grasp_size, grasp_data, possible_grasps);
}

bool GraspGenerator::generateGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width, double height,
                                    double max_grasp_size, const moveit_grasps::GraspDataPtr grasp_data,
                                    std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // Generate grasps over axes that aren't too wide to grip

  // Most default type of grasp is X axis
  if (depth <= max_grasp_size ) // depth = size along x-axis
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator","Generating grasps around x-axis of cuboid");
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, X_AXIS, grasp_data, possible_grasps);
  }

  if (width <= max_grasp_size ) // width = size along y-axis
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator","Generating grasps around y-axis of cuboid");
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Y_AXIS, grasp_data, possible_grasps);
  }

  if (height <= max_grasp_size ) // height = size along z-axis
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator","Generating grasps around z-axis of cuboid");
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Z_AXIS, grasp_data, possible_grasps);
  }

  if (!possible_grasps.size())
    ROS_WARN_STREAM_NAMED("grasp_generator","Generated 0 grasps");
  else
    ROS_INFO_STREAM_NAMED("grasp_generator","Generated " << possible_grasps.size() << " grasps");

  // Visualize animated grasps that have been generated
  if (show_prefiltered_grasps_)
  {
    ROS_DEBUG_STREAM_NAMED("grasp_generator","Animating all generated (candidate) grasps before filtering");
    visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data->ee_jmg_, show_prefiltered_grasps_speed_);
  }

  return true;
}

bool GraspGenerator::generateCuboidAxisGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width, double height, grasp_axis_t axis,
                                              const moveit_grasps::GraspDataPtr grasp_data, std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // create transform from object to world frame (/base_link)
  Eigen::Affine3d object_global_transform = cuboid_pose;

  // grasp parameters

  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","generating reusable motions and msgs");

  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();
  pre_grasp_approach.desired_distance = grasp_data->finger_to_palm_depth_;
  pre_grasp_approach.min_distance = grasp_data->finger_to_palm_depth_;

  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();
  post_grasp_retreat.desired_distance = grasp_data->finger_to_palm_depth_;
  post_grasp_retreat.min_distance = grasp_data->finger_to_palm_depth_;

  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data->base_link_;

  // grasp generator loop
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","offsetting grasp points by gripper finger length, " << grasp_data->finger_to_palm_depth_);
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","Translate gripper by " << grasp_data->grasp_pose_to_eef_pose_.translation() );
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","Rotate gripper by " << grasp_data->grasp_pose_to_eef_pose_.rotation() );
  double radius = grasp_data->finger_to_palm_depth_;

  moveit_msgs::Grasp new_grasp;
  static int grasp_id = 0;
  double grasp_score;

  Eigen::Affine3d grasp_pose;
  Eigen::Affine3d base_grasp_pose;

  double dx = cuboid_pose.translation()[0];
  double dy = cuboid_pose.translation()[1];
  double dz = cuboid_pose.translation()[2];

  Eigen::Vector3d grasp_translation;
  Eigen::ArrayXXf grasp_points;

  new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
  grasp_id++;

  // pre-grasp and grasp postures
  new_grasp.pre_grasp_posture = grasp_data->pre_grasp_posture_;
  new_grasp.grasp_posture = grasp_data->grasp_posture_;


  // Approach and retreat
  // aligned with pose (aligned with grasp pose z-axis
  // TODO:: Currently the pre/post approach/retreat are not being used. Either remove or make it robot agnostic.
  // It currently being loaded with the assumption that z-axis is pointing away from object.
  Eigen::Vector3d approach_vector;
  approach_vector = grasp_pose * Eigen::Vector3d::UnitZ();
  approach_vector.normalize();

  pre_grasp_approach.direction.header.frame_id = grasp_data->parent_link_name_;
  pre_grasp_approach.direction.vector.x = 0;
  pre_grasp_approach.direction.vector.y = 0;
  pre_grasp_approach.direction.vector.z = 1;
  new_grasp.pre_grasp_approach = pre_grasp_approach;

  post_grasp_retreat.direction.header.frame_id = grasp_data->parent_link_name_;
  post_grasp_retreat.direction.vector.x = 0;
  post_grasp_retreat.direction.vector.y = 0;
  post_grasp_retreat.direction.vector.z = -1;
  new_grasp.post_grasp_retreat = post_grasp_retreat;

  switch(axis)
  {
    case X_AXIS:
      // will rotate around x-axis testing grasps
      grasp_points = generateCuboidGraspPoints(height, width, radius);

      base_grasp_pose = cuboid_pose
        * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ());
      break;

    case Y_AXIS:
      // will rotate around y-axis testing grasps
      grasp_points = generateCuboidGraspPoints(height, depth, radius);

      base_grasp_pose = cuboid_pose *
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY());
      break;

    case Z_AXIS:
      // will rotate around z-axis testing grasps
      grasp_points = generateCuboidGraspPoints(depth, width, radius);

      base_grasp_pose = cuboid_pose *
        Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX());
      break;

    default:
      ROS_WARN_STREAM_NAMED("cuboid_axis_grasps","grasp axis may not be defined properly");
      break;
  }

  for (size_t i = 0; i < grasp_points.size() / 3; i++ )
  {
    grasp_pose = base_grasp_pose;

    // move to grasp position
    grasp_translation = grasp_pose * Eigen::Vector3d(grasp_points(i,0), 0, grasp_points(i,1)) -
      Eigen::Vector3d(dx,dy,dz);
    grasp_pose.translation() += grasp_translation;

    // Visualize lines for getting gripper rotation angle
    Eigen::Vector3d cuboidCenter = cuboid_pose.translation();
    Eigen::Vector3d poseCenter = grasp_pose.translation();
    Eigen::Vector3d zAxis = grasp_pose * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d yAxis = grasp_pose * Eigen::Vector3d::UnitY();

    // now rotate to point toward center of cuboid
    Eigen::Vector3d gripper_z = zAxis - poseCenter;
    gripper_z.normalized();
    Eigen::Vector3d to_cuboid = cuboidCenter - poseCenter;
    to_cuboid.normalized();
    Eigen::Vector3d cross_prod = gripper_z.cross(to_cuboid);

    for (size_t j = 0; j < 3; j++)
    {
      // check for possible negative zero values...
      if (std::abs(cross_prod[j]) < 0.000001)
        cross_prod[j] = 0;
    }

    double dot = cross_prod.dot(yAxis - poseCenter);
    double rotation_angle = acos( gripper_z.normalized().dot(to_cuboid.normalized()));

    // check sign of rotation angle
    if (dot < 0)
      rotation_angle *= -1;

    ROS_DEBUG_STREAM_NAMED("cuboid","");
    ROS_DEBUG_STREAM_NAMED("cuboid","yAxis      = " << yAxis[0] << " " << yAxis[1] << " " << yAxis[2]);
    ROS_DEBUG_STREAM_NAMED("cuboid","cross_prod = " << cross_prod[0] << " " << cross_prod[1] << " " << cross_prod[2]);
    ROS_DEBUG_STREAM_NAMED("cuboid","yAxis . cross_prod = " << dot);

    grasp_pose = grasp_pose * Eigen::AngleAxisd(rotation_angle, Eigen::Vector3d::UnitY());

    if (verbose_)
    {
      // collect all markers before publishing to rviz
      visual_tools_->enableBatchPublishing(true);

      // show generated grasp pose
      visual_tools_->publishAxis(grasp_pose, 0.05, 0.005);
      visual_tools_->publishSphere(grasp_pose.translation(), rviz_visual_tools::PINK, 0.01);
    }

    // translate and rotate gripper to match standard orientation
    // origin on palm, z pointing outward, x perp to gripper close, y parallel to gripper close direction
    // Transform the grasp pose
    grasp_pose = grasp_pose * grasp_data->grasp_pose_to_eef_pose_;

    tf::poseEigenToMsg(grasp_pose, grasp_pose_msg.pose);
    new_grasp.grasp_pose = grasp_pose_msg;
    possible_grasps.push_back(new_grasp);

    if (verbose_)
    {
      // show gripper center and grasp direction
      visual_tools_->publishXArrow(new_grasp.grasp_pose.pose, rviz_visual_tools::RED, rviz_visual_tools::SMALL, 0.05);
      //visual_tools_->publishYArrow(new_grasp.grasp_pose.pose, rviz_visual_tools::GREEN, rviz_visual_tools::SMALL, 0.05);
      visual_tools_->publishZArrow(new_grasp.grasp_pose.pose, rviz_visual_tools::BLUE, rviz_visual_tools::SMALL, 0.05);
      visual_tools_->publishBlock(new_grasp.grasp_pose.pose, rviz_visual_tools::PINK, 0.01);

      // Send markers to Rviz
      visual_tools_->triggerBatchPublishAndDisable();
      ros::Duration(0.05).sleep();
    }
  }
}

geometry_msgs::PoseStamped GraspGenerator::getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link)
{
  // Grasp Pose Variables
  geometry_msgs::PoseStamped grasp_pose = grasp.grasp_pose;
  Eigen::Affine3d grasp_pose_eigen;
  tf::poseMsgToEigen(grasp_pose.pose, grasp_pose_eigen);

  // Get pre-grasp pose first
  geometry_msgs::PoseStamped pre_grasp_pose;
  Eigen::Affine3d pre_grasp_pose_eigen = grasp_pose_eigen; // Copy original grasp pose to pre-grasp pose

  // Approach direction variables
  Eigen::Vector3d pre_grasp_approach_direction_local;

  // The direction of the pre-grasp
  // Calculate the current animation position based on the percent
  Eigen::Vector3d pre_grasp_approach_direction = Eigen::Vector3d(
                                                                 -1 * grasp.pre_grasp_approach.direction.vector.x * grasp.pre_grasp_approach.desired_distance,
                                                                 -1 * grasp.pre_grasp_approach.direction.vector.y * grasp.pre_grasp_approach.desired_distance,
                                                                 -1 * grasp.pre_grasp_approach.direction.vector.z * grasp.pre_grasp_approach.desired_distance
                                                                 );

  // Decide if we need to change the approach_direction to the local frame of the end effector orientation
  if( grasp.pre_grasp_approach.direction.header.frame_id == ee_parent_link )
  {
    // Apply/compute the approach_direction vector in the local frame of the grasp_pose orientation
    pre_grasp_approach_direction_local = grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }
  else
  {
    pre_grasp_approach_direction_local = pre_grasp_approach_direction; //grasp_pose_eigen.rotation() * pre_grasp_approach_direction;
  }

  // Update the grasp matrix usign the new locally-framed approach_direction
  pre_grasp_pose_eigen.translation() += pre_grasp_approach_direction_local;

  // Convert eigen pre-grasp position back to regular message
  tf::poseEigenToMsg(pre_grasp_pose_eigen, pre_grasp_pose.pose);

  // Copy original header to new grasp
  pre_grasp_pose.header = grasp_pose.header;

  return pre_grasp_pose;
}

void GraspGenerator::publishGraspArrow(geometry_msgs::Pose grasp, const GraspDataPtr grasp_data,
                                       const rviz_visual_tools::colors &color, double approach_length)
{
  //Eigen::Affine3d eigen_grasp_pose;
  // Convert each grasp back to forward-facing error (undo end effector custom rotation)
  //tf::poseMsgToEigen(grasp, eigen_grasp_pose);
  //eigen_grasp_pose = eigen_grasp_pose * grasp_data->grasp_pose_to_eef_pose_.inverse();

  //visual_tools_->publishArrow(eigen_grasp_pose, color, rviz_visual_tools::REGULAR);
  visual_tools_->publishArrow(grasp, color, rviz_visual_tools::REGULAR);
}

Eigen::ArrayXXf GraspGenerator::generateCuboidGraspPoints(double length, double width, double radius)
{
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points","generating possible grasp points around cuboid");

  /*
   * Create equally spaced points around cuboid
   */

  // choose the larger of the two and make angular increments about equal
  //double delta = (2 * length + 2 * width + 2 * M_PI * radius) / number_grasp_points_;
  if (mm_between_grasps_ < MIN_GRASP_DISTANCE)
  {
    mm_between_grasps_ = MIN_GRASP_DISTANCE;
    ROS_WARN_STREAM_NAMED("cuboid_grasp_points","mm_between_grasps_ < MIN_GRASP_DISTANCE ( " << MIN_GRASP_DISTANCE << ")");
  }
  double delta = mm_between_grasps_ / 1000; // mm to m

  size_t top_bottom_array_size = length / delta + 1;
  if (top_bottom_array_size <= 2)
    top_bottom_array_size = 0;
  double top_bottom_delta = length / (top_bottom_array_size - 1); // delta for top/bottom of cuboid

  size_t left_right_array_size = width / delta + 1;
  if (left_right_array_size <= 2)
    left_right_array_size = 0;
  double left_right_delta = width / (left_right_array_size - 1); // delta for sides of cuboid

  double corner_delta =  delta / radius;
  size_t corner_array_size = (M_PI / 2) / corner_delta + 1;
  corner_delta = (M_PI / 2) / (corner_array_size - 1.0) ; // update delta due to integer rounding

  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points.deltas",
                         "top_bottom (delta,array size) = " << top_bottom_delta << ", " << top_bottom_array_size);
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points.deltas",
                         "left_right (delta,array size) = " << left_right_delta << ", " << left_right_array_size);
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points.deltas",
                         "corner (delta,array size) = " << corner_delta << ", " << corner_array_size);

  Eigen::ArrayXXf top = Eigen::ArrayXXf::Zero(top_bottom_array_size,3);
  Eigen::ArrayXXf bottom = Eigen::ArrayXXf::Zero(top_bottom_array_size,3);
  Eigen::ArrayXXf right = Eigen::ArrayXXf::Zero(left_right_array_size,3);
  Eigen::ArrayXXf left = Eigen::ArrayXXf::Zero(left_right_array_size,3);
  Eigen::ArrayXXf corners = Eigen::ArrayXXf::Zero(corner_array_size, 3);

  double cornerX = length / 2.0;
  double cornerY = width / 2.0;

  // empty points array, don't keep end points since rounded corners will overlap
  size_t points_size = 2 * top_bottom_array_size + 2 * left_right_array_size + corner_array_size * 4;
  if ( top_bottom_array_size != 0 ) // check that edge is large enough to contain points
    points_size -= 4;
  if ( left_right_array_size != 0 ) // check that edge is large enough to contain points
    points_size -= 4;

  Eigen::ArrayXXf points = Eigen::ArrayXXf::Zero(points_size,3);
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "number of points = " << points.size() / 3);

  // create points along side of cuboid and add them to array
  int offset = 0;
  int left_right_offset = 0;
  if (top_bottom_array_size != 0)
  {
    ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "generating top/bottom points");
    for (size_t i = 0; i < top_bottom_array_size; i++)
    {
      top.row(i)    << -length / 2 + i * top_bottom_delta , width / 2 + radius, 0;
      bottom.row(i) << -length / 2 + i * top_bottom_delta , -width / 2 - radius, 0;
    }
    offset = top_bottom_array_size - 2;
    ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "offset = " << offset);
    points.block(0,      0, offset, 3) =    top.block(1, 0, offset, 3);
    points.block(offset, 0, offset, 3) = bottom.block(1, 0, offset, 3);
  }

  offset *= 2;
  if (left_right_array_size != 0)
  {
    ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "generating left/right points");
    for (size_t i = 0; i < left_right_array_size; i++)
    {
      left.row(i)    <<  length / 2 + radius, -width / 2 + i * left_right_delta, 0;
      right.row(i) << -length / 2 - radius, -width / 2 + i * left_right_delta, 0;
    }
    ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "offset = " << offset);
    left_right_offset = left_right_array_size - 2;
    ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "left_right_offset = " << left_right_offset);
    points.block(offset,                     0, left_right_offset, 3) = left.block(1,0,left_right_offset,3);
    points.block(offset + left_right_offset, 0, left_right_offset, 3) = right.block(1,0,left_right_offset,3);
  }

  // create points along corners and add them to array
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "generating corner points");
  offset += 2 * left_right_offset;
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "offset = " << offset);
  for (size_t j = 0; j < 4; j++)
  {
    // get corner coordinates
    switch( j )
    {
      case 0:
        // first corner doesn't change values
        break;
      case 1:
        cornerX *= -1;
        break;
      case 2:
        cornerY *= -1;
        break;
      case 3:
        cornerX *= -1;
        break;
      default:
        ROS_WARN_STREAM_NAMED("cuboid_grasp_points", "fell through on corner selection. cornerX = "
                              << cornerX << ", cornerY = " << cornerY);
        break;
    }
    ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "corner = " << cornerX << ", " << cornerY);

    // translate to corner and rotate
    for (size_t i = 0; i < corner_array_size; i++)
    {
      corners.row(i) <<
        cornerX + radius * cos((j * M_PI / 2) + i * corner_delta),
        cornerY + radius * sin((j * M_PI / 2) + i * corner_delta), 0;
    }
    points.block(offset, 0, corner_array_size, 3) = corners;
    offset += corner_array_size;

  }

  // get angle from centroid to grasp point
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "calculating angles...");
  for (size_t i = 0; i < points.size() / 3; i++)
  {
    points(i,2) = atan2(points(i,1),points(i,0));
  }

  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points","top \n = " << top);
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points","bottom \n = " << bottom);
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points","left \n = " << left);
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points","right \n = " << right);
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points","points \n = " << points);

  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points", "Generated " << points.size() / 3 << " possible grasp points");

  return points;
}

bool GraspGenerator::getBoundingBoxFromMesh(const shape_msgs::Mesh& mesh_msg, Eigen::Affine3d& cuboid_pose,
                                            double& depth, double& width, double& height)
{
  int num_vertices = mesh_msg.vertices.size();
  ROS_DEBUG_STREAM_NAMED("bbox","num triangles = " << mesh_msg.triangles.size());
  ROS_DEBUG_STREAM_NAMED("bbox","num vertices = " << num_vertices);

  // calculate centroid and moments of inertia
  // NOTE: Assimp adds verticies to imported meshes, which is not accounted for in the MOI and CG calculations
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  double Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
  Ixx = 0; Iyy = 0; Izz = 0; Ixy = 0; Ixz = 0; Iyz = 0;

  for (int i = 0; i < num_vertices; i++)
  {
    // centroid sum
    point << mesh_msg.vertices[i].x, mesh_msg.vertices[i].y, mesh_msg.vertices[i].z;
    centroid += point;

    // moments of inertia sum
    Ixx += point[1] * point[1] + point[2] * point[2];
    Iyy += point[0] * point[0] + point[2] * point[2];
    Izz += point[0] * point[0] + point[1] * point[1];
    Ixy += point[0] * point[1];
    Ixz += point[0] * point[2];
    Iyz += point[1] * point[2];

  }

  // final centroid calculation
  for (int i = 0; i < 3; i++)
  {
    centroid[i] /= num_vertices;
  }
  ROS_DEBUG_STREAM_NAMED("bbox","centroid = \n" << centroid);

  if (verbose_)
    visual_tools_->publishSphere(centroid, rviz_visual_tools::PINK, 0.01);

  // Solve for principle axes of inertia
  Eigen::Matrix3d inertia_axis_aligned;
  inertia_axis_aligned.row(0) <<  Ixx, -Ixy, -Ixz;
  inertia_axis_aligned.row(1) << -Ixy,  Iyy, -Iyz;
  inertia_axis_aligned.row(2) << -Ixz, -Iyz,  Izz;

  ROS_DEBUG_STREAM_NAMED("bbox","inertia_axis_aligned = \n" << inertia_axis_aligned);

  Eigen::EigenSolver<Eigen::MatrixXd> es(inertia_axis_aligned);

  ROS_DEBUG_STREAM_NAMED("bbox","eigenvalues = \n" << es.eigenvalues());
  ROS_DEBUG_STREAM_NAMED("bbox","eigenvectors = \n" << es.eigenvectors());

  Eigen::Vector3d axis_1 = es.eigenvectors().col(0).real();
  Eigen::Vector3d axis_2 = es.eigenvectors().col(1).real();
  Eigen::Vector3d axis_3 = es.eigenvectors().col(2).real();

  // Test if eigenvectors are right-handed
  Eigen::Vector3d w = axis_1.cross(axis_2) - axis_3;
  double epsilon = 0.000001;
  if ( !(std::abs(w(0)) < epsilon && std::abs(w(1)) < epsilon && std::abs(w(2)) < epsilon) )
  {
    axis_3 *= -1;
    ROS_DEBUG_STREAM_NAMED("bbox","eigenvectors are left-handed, multiplying v3 by -1");
  }

  // assumes msg was given wrt world... probably needs better name
  Eigen::Affine3d world_to_mesh_transform = Eigen::Affine3d::Identity();
  world_to_mesh_transform.linear() << axis_1, axis_2, axis_3;
  world_to_mesh_transform.translation() = centroid;

  // Transform and get bounds
  Eigen::Vector3d min;
  Eigen::Vector3d max;
  for (int i = 0; i < 3; i++)
  {
    min(i)=std::numeric_limits<double>::max();
    max(i)=std::numeric_limits<double>::min();
  }

  for (int i = 0; i < num_vertices; i++)
  {
    point << mesh_msg.vertices[i].x, mesh_msg.vertices[i].y, mesh_msg.vertices[i].z;
    point = world_to_mesh_transform.inverse() * point;
    for (int j = 0; j < 3; j++)
    {
      if (point(j) < min(j))
        min(j) = point(j);

      if (point(j) > max(j))
        max(j) = point(j);
    }
  }
  ROS_DEBUG_STREAM_NAMED("bbox","min = \n" << min);
  ROS_DEBUG_STREAM_NAMED("bbox","max = \n" << max);

  // points
  Eigen::Vector3d p[8];

  p[0] << min(0), min(1), min(2);
  p[1] << max(0), min(1), min(2);
  p[2] << min(0), max(1), min(2);
  p[3] << max(0), max(1), min(2);

  p[4] << min(0), min(1), max(2);
  p[5] << max(0), min(1), max(2);
  p[6] << min(0), max(1), max(2);
  p[7] << max(0), max(1), max(2);

  if (verbose_)
  {
    for (int i = 0; i < 8; i++)
      visual_tools_->publishSphere(world_to_mesh_transform * p[i],rviz_visual_tools::YELLOW,0.01);
  }

  depth = max(0) - min(0);
  width = max(1) - min(1);
  height = max(2) - min(2);
  ROS_DEBUG_STREAM_NAMED("bbox","bbox size = " << depth << ", " << width << ", " << height);

  Eigen::Vector3d translation;
  translation << (min(0) + max(0)) / 2.0, (min(1) + max(1)) / 2.0, (min(2) + max(2)) / 2.0;
  ROS_DEBUG_STREAM_NAMED("bbox","bbox origin = \n" << translation);
  cuboid_pose = world_to_mesh_transform;
  cuboid_pose.translation() = world_to_mesh_transform * translation;

  if (verbose_)
  {
    visual_tools_->publishCuboid(visual_tools_->convertPose(cuboid_pose),
                                 depth,width,height,rviz_visual_tools::TRANSLUCENT);
    visual_tools_->publishAxis(world_to_mesh_transform);
  }

  return true;
}


} // namespace
