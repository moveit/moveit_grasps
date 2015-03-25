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

#include <moveit_grasps/grasps.h>

namespace moveit_grasps
{

// Constructor
Grasps::Grasps(moveit_visual_tools::MoveItVisualToolsPtr visual_tools, bool verbose) :
  visual_tools_(visual_tools),
  verbose_(verbose)
{
  ROS_DEBUG_STREAM_NAMED("grasps","Loaded grasp generator");
  number_grasp_points_ = 50;
}

// Deconstructor
Grasps::~Grasps()
{
}

// Create all possible grasp positions for a object
bool Grasps::generateBlockGrasps(const Eigen::Affine3d& object_pose, const GraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Calculate grasps in two axis in both directions
  generateAxisGrasps( object_pose, X_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // got no grasps with this alone
  generateAxisGrasps( object_pose, X_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gives some grasps... looks ugly
  generateAxisGrasps( object_pose, Y_AXIS, DOWN, HALF, 0, grasp_data, possible_grasps); // GOOD ONES!
  generateAxisGrasps( object_pose, Y_AXIS, UP,   HALF, 0, grasp_data, possible_grasps); // gave a grasp from top... bad

  return true;
}

// Create grasp positions in one axis
bool Grasps::generateAxisGrasps(
  const Eigen::Affine3d& object_pose,
  grasp_axis_t axis,
  grasp_direction_t direction,
  grasp_rotation_t rotation,
  double hand_roll,
  const GraspData& grasp_data,
  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // ---------------------------------------------------------------------------------------------
  // Create a transform from the object's frame (center of object) to /base_link
  object_global_transform_ = object_pose;

  // ---------------------------------------------------------------------------------------------
  // Grasp parameters

  // Create re-usable approach motion
  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();
  pre_grasp_approach.desired_distance = grasp_data.finger_to_palm_depth_ + 0.1; // The distance the origin of a robot link needs to travel
  pre_grasp_approach.min_distance = grasp_data.finger_to_palm_depth_; // half of the desired? Untested.

  // Create re-usable retreat motion
  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();
  post_grasp_retreat.desired_distance = grasp_data.finger_to_palm_depth_ + 0.1; // The distance the origin of a robot link needs to travel
  post_grasp_retreat.min_distance = grasp_data.finger_to_palm_depth_; // half of the desired? Untested.

  // Create re-usable blank pose
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // ---------------------------------------------------------------------------------------------
  // Angle calculations
  double radius = grasp_data.grasp_depth_; //0.12
  double xb;
  double yb = 0.0; // stay in the y plane of the object
  double zb;
  double theta1 = 0.0; // Where the point is located around the object
  double theta2 = 0.0; // UP 'direction'

  // Gripper direction (UP/DOWN) rotation. UP set by default
  if( direction == DOWN )
  {
    theta2 = M_PI;
  }

  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------
  // Begin Grasp Generator Loop
  // ---------------------------------------------------------------------------------------------
  // ---------------------------------------------------------------------------------------------

  /* Developer Note:
   * Create angles 180 degrees around the chosen axis at given resolution
   * We create the grasps in the reference frame of the object, then later convert it to the base link
   */
  for(int i = 0; i <= grasp_data.angle_resolution_; ++i)
  {
    // Create a Grasp message
    moveit_msgs::Grasp new_grasp;

    // Calculate grasp pose
    xb = radius*cos(theta1);
    zb = radius*sin(theta1);

    Eigen::Affine3d grasp_pose;

    switch(axis)
    {
      case X_AXIS:
        grasp_pose = Eigen::AngleAxisd(theta1, Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( yb, xb ,zb);

        break;
      case Y_AXIS:
        grasp_pose =
          Eigen::AngleAxisd(M_PI - theta1, Eigen::Vector3d::UnitY())
          *Eigen::AngleAxisd(theta2, Eigen::Vector3d::UnitX()); // Flip 'direction'

        grasp_pose.translation() = Eigen::Vector3d( xb, yb ,zb);

        break;
      case Z_AXIS:
        ROS_ERROR_STREAM_NAMED("grasp","Z Axis not implemented!");
        return false;

        break;
    }

    /* The estimated probability of success for this grasp, or some other measure of how "good" it is.
     * Here we base bias the score based on how far the wrist is from the surface, preferring a greater
     * distance to prevent wrist/end effector collision with the table
     */
    double score = sin(theta1);
    new_grasp.grasp_quality = std::max(score,0.1); // don't allow score to drop below 0.1 b/c all grasps are ok

    // Calculate the theta1 for next time
    if (rotation == HALF)
      theta1 += M_PI / grasp_data.angle_resolution_;
    else
    {
      theta1 += 2*M_PI / grasp_data.angle_resolution_;
    }

    // A name for this grasp
    static int grasp_id = 0;
    new_grasp.id = "Grasp" + boost::lexical_cast<std::string>(grasp_id);
    ++grasp_id;

    // PreGrasp and Grasp Postures --------------------------------------------------------------------------

    // The internal posture of the hand for the pre-grasp only positions are used
    new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;

    // The internal posture of the hand for the grasp positions and efforts are used
    new_grasp.grasp_posture = grasp_data.grasp_posture_;

    // Grasp ------------------------------------------------------------------------------------------------

    // HACK - turn object 90*
    grasp_pose = Eigen::AngleAxisd(-0.5*M_PI, Eigen::Vector3d::UnitY()) * grasp_pose;

    // DEBUG - show original grasp pose before tranform to gripper frame
    if( verbose_ )
    {
      tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);
      visual_tools_->publishArrow(grasp_pose_msg.pose, rviz_visual_tools::GREEN);
    }

    // ------------------------------------------------------------------------
    // Optionally roll wrist with respect to object pose
    Eigen::Affine3d roll_gripper;
    roll_gripper = Eigen::AngleAxisd(hand_roll, Eigen::Vector3d::UnitX());
    grasp_pose = grasp_pose * roll_gripper;

    // ------------------------------------------------------------------------
    // Change grasp to frame of reference of this custom end effector

    // Transform the grasp pose
    grasp_pose = grasp_pose * grasp_data.grasp_pose_to_eef_pose_;

    // ------------------------------------------------------------------------
    // Convert pose to global frame (base_link)
    tf::poseEigenToMsg(object_global_transform_ * grasp_pose, grasp_pose_msg.pose);

    // The position of the end-effector for the grasp relative to a reference frame (that is always specified elsewhere, not in this message)
    new_grasp.grasp_pose = grasp_pose_msg;

    // Other ------------------------------------------------------------------------------------------------

    // the maximum contact force to use while grasping (<=0 to disable)
    new_grasp.max_contact_force = 0;

    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------
    // Approach and retreat
    // TODO:: Currently the pre/post approach/retreat are not being used. Either remove or make it robot agnostic.
    // It currently being loaded with the assumption that z-axis is pointing away from object.
    // -------------------------------------------------------------------------------------------------------
    // -------------------------------------------------------------------------------------------------------

    // Straight down ---------------------------------------------------------------------------------------
    // With respect to the base link/world frame

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.base_link_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = -1; // Approach direction (negative z axis)  // TODO: document this assumption
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.base_link_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = 1; // Retreat direction (pos z axis)
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

    // Angled with pose -------------------------------------------------------------------------------------
    // Approach with respect to end effector orientation

    // Approach
    pre_grasp_approach.direction.header.frame_id = grasp_data.parent_link_name_;
    pre_grasp_approach.direction.vector.x = 0;
    pre_grasp_approach.direction.vector.y = 0;
    pre_grasp_approach.direction.vector.z = 1;
    new_grasp.pre_grasp_approach = pre_grasp_approach;

    // Retreat
    post_grasp_retreat.direction.header.frame_id = grasp_data.parent_link_name_;
    post_grasp_retreat.direction.vector.x = 0;
    post_grasp_retreat.direction.vector.y = 0;
    post_grasp_retreat.direction.vector.z = -1;
    new_grasp.post_grasp_retreat = post_grasp_retreat;

    // Add to vector
    possible_grasps.push_back(new_grasp);

  }

  ROS_INFO_STREAM_NAMED("grasp", "Generated " << possible_grasps.size() << " grasps." );

  return true;
}

geometry_msgs::PoseStamped Grasps::getPreGraspPose(const moveit_msgs::Grasp &grasp, const std::string &ee_parent_link)
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

void Grasps::publishGraspArrow(geometry_msgs::Pose grasp, const GraspData& grasp_data,
                               const rviz_visual_tools::colors &color, double approach_length)
{
  //Eigen::Affine3d eigen_grasp_pose;
  // Convert each grasp back to forward-facing error (undo end effector custom rotation)
  //tf::poseMsgToEigen(grasp, eigen_grasp_pose);
  //eigen_grasp_pose = eigen_grasp_pose * grasp_data.grasp_pose_to_eef_pose_.inverse();

  //visual_tools_->publishArrow(eigen_grasp_pose, color, rviz_visual_tools::REGULAR);
  visual_tools_->publishArrow(grasp, color, rviz_visual_tools::REGULAR);
}

Eigen::ArrayXXf Grasps::generateCuboidGraspPoints(double length, double width, double radius)
{
  ROS_DEBUG_STREAM_NAMED("cuboid_grasp_points","generating possible grasp points around cuboid");

  /*
   * Create equally spaced points around cuboid
   */

  // choose the larger of the two and make angular increments about equal
  double delta = (2 * length + 2 * width + 2 * M_PI * radius) / number_grasp_points_;

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

bool Grasps::generateCuboidAxisGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width, double height, grasp_axis_t axis,
                                      const moveit_grasps::GraspData& grasp_data, std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // create transform from object to world frame (/base_link)
  Eigen::Affine3d object_global_transform = cuboid_pose;

  // grasp parameters

  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","generating reusable motions and msgs");

  moveit_msgs::GripperTranslation pre_grasp_approach;
  pre_grasp_approach.direction.header.stamp = ros::Time::now();
  pre_grasp_approach.desired_distance = grasp_data.finger_to_palm_depth_ + 0.1;
  pre_grasp_approach.min_distance = grasp_data.finger_to_palm_depth_;

  moveit_msgs::GripperTranslation post_grasp_retreat;
  post_grasp_retreat.direction.header.stamp = ros::Time::now();
  post_grasp_retreat.desired_distance = grasp_data.finger_to_palm_depth_ + 0.1;
  post_grasp_retreat.min_distance = grasp_data.finger_to_palm_depth_;
    
  geometry_msgs::PoseStamped grasp_pose_msg;
  grasp_pose_msg.header.stamp = ros::Time::now();
  grasp_pose_msg.header.frame_id = grasp_data.base_link_;

  // grasp generator loop
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","offsetting grasp points by gripper finger length, " << grasp_data.finger_to_palm_depth_);
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","Translate gripper by " << grasp_data.grasp_pose_to_eef_pose_.translation() );
  ROS_DEBUG_STREAM_NAMED("cuboid_axis_grasps","Rotate gripper by " << grasp_data.grasp_pose_to_eef_pose_.rotation() );
  double radius = grasp_data.finger_to_palm_depth_; 

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
  new_grasp.pre_grasp_posture = grasp_data.pre_grasp_posture_;
  new_grasp.grasp_posture = grasp_data.grasp_posture_;


  // Approach and retreat
  // aligned with pose (aligned with grasp pose z-axis
  // TODO:: Currently the pre/post approach/retreat are not being used. Either remove or make it robot agnostic.
  // It currently being loaded with the assumption that z-axis is pointing away from object.
  Eigen::Vector3d approach_vector;
  approach_vector = grasp_pose * Eigen::Vector3d::UnitZ();
  approach_vector.normalize();

  pre_grasp_approach.direction.header.frame_id = grasp_data.parent_link_name_;
  pre_grasp_approach.direction.vector.x = 0; 
  pre_grasp_approach.direction.vector.y = 0; 
  pre_grasp_approach.direction.vector.z = 1;
  new_grasp.pre_grasp_approach = pre_grasp_approach;
   
  post_grasp_retreat.direction.header.frame_id = grasp_data.parent_link_name_;
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
    grasp_pose = grasp_pose * grasp_data.grasp_pose_to_eef_pose_;

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

bool Grasps::generateCuboidGrasps(const Eigen::Affine3d& cuboid_pose, double depth, double width, double height, 
                                  double max_grasp_size, const moveit_grasps::GraspData& grasp_data, 
                                  std::vector<moveit_msgs::Grasp>& possible_grasps)
{
  // generate grasps over axes that aren't too wide to grip with Open Hand
  
  // Most default type of grasp is X axis
  if (depth <= max_grasp_size ) // depth = size along x-axis
  {
    ROS_DEBUG_STREAM_NAMED("cuboid_grasps","Generating grasps around x-axis of cuboid");
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, X_AXIS, grasp_data, possible_grasps);
  }

  if (width <= max_grasp_size ) // width = size along y-axis
  {
    ROS_DEBUG_STREAM_NAMED("cuboid_grasps","Generating grasps around y-axis of cuboid");
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Y_AXIS, grasp_data, possible_grasps);
  }

  if (height <= max_grasp_size ) // height = size along z-axis
  {
    ROS_DEBUG_STREAM_NAMED("cuboid_grasps","Generating grasps around z-axis of cuboid");
    generateCuboidAxisGrasps(cuboid_pose, depth, width, height, Z_AXIS, grasp_data, possible_grasps);
  }    
    
}

} // namespace
