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

/* Author: Andy McEvoy
   Desc:   creates a vizualization of all the poses used in the graping pipeline
*/

#include <moveit_grasps/grasp_generator.h>

namespace moveit_grasps
{
// Size and location for randomly generated cuboids
static const double CUBOID_MIN_SIZE = 0.02;
static const double CUBOID_MAX_SIZE = 0.07;
static const double CUBOID_WORKSPACE_MIN_X = 0.01;
static const double CUBOID_WORKSPACE_MAX_X = 0.5;
static const double CUBOID_WORKSPACE_MIN_Y = -0.5;
static const double CUBOID_WORKSPACE_MAX_Y = 0.5;
static const double CUBOID_WORKSPACE_MIN_Z = 0.0;
static const double CUBOID_WORKSPACE_MAX_Z = 1.0;

class GraspPosesVisualizer
{
private:
  ros::NodeHandle nh_;

  // cuboid dimensions
  double depth_;
  double width_;
  double height_;
  geometry_msgs::Pose cuboid_pose_;
  moveit_grasps::GraspGeneratorPtr grasp_generator_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  std::vector<GraspCandidatePtr> grasp_candidates_;
  moveit_grasps::GraspDataPtr grasp_data_;

  // TODO: read in from param

  // arm description
  std::string ee_group_name_;
  std::string planning_group_name_;

public:
  // Constructor
  GraspPosesVisualizer(bool verbose) : nh_("~")
  {
    // get arm parameters
    nh_.param("ee_group_name", ee_group_name_, std::string("left_hand"));

    ROS_INFO_STREAM_NAMED("init", "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("init", "Planning Group: " << planning_group_name_);

    // set up rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();

    // Load grasp data
    grasp_data_.reset(new GraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()));

    // load grasp generator
    grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_, verbose));

    // initialize cuboid size
    depth_ = CUBOID_MIN_SIZE;
    width_ = CUBOID_MIN_SIZE;
    height_ = CUBOID_MIN_SIZE;
    // Seed random
    srand(ros::Time::now().toSec());

    visual_tools_->deleteAllMarkers();

    ROS_INFO_STREAM_NAMED("viz_test", "\n************* \nStarting Vizualization"
                                          << "\n*************");

    ROS_INFO_STREAM_NAMED("viz_test", "generating random cuboid");
    generateRandomCuboid(cuboid_pose_, depth_, width_, height_);

    Eigen::Affine3d display_pose;
    bool text = false;

    // SHOW OBJECT POSE
    visual_tools_->publishCuboid(cuboid_pose_, depth_, width_, height_);
    visual_tools_->publishAxis(cuboid_pose_, 0.05, 0.005);
    visual_tools_->publishText(cuboid_pose_, "Object Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XSMALL, text);
    grasp_candidates_.clear();
    grasp_generator_->generateGrasps(visual_tools_->convertPose(cuboid_pose_), depth_, width_, height_, grasp_data_,
                                     grasp_candidates_);

    // SHOW EE GRASP POSE
    Eigen::Affine3d ee_pose = visual_tools_->convertPose(grasp_candidates_[50]->grasp_.grasp_pose.pose);
    visual_tools_->publishAxis(ee_pose, 0.05, 0.005);
    visual_tools_->publishSphere(ee_pose.translation(), rviz_visual_tools::GREEN, 0.02);
    visual_tools_->publishText(ee_pose, "EE Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XSMALL, text);

    // SHOW GRASP POSE
    Eigen::Affine3d grasp_pose;
    grasp_pose = ee_pose * grasp_data_->grasp_pose_to_eef_pose_.inverse();
    visual_tools_->publishAxis(grasp_pose, 0.05, 0.005);
    visual_tools_->publishSphere(grasp_pose.translation(), rviz_visual_tools::LIME_GREEN, 0.02);
    visual_tools_->publishText(grasp_pose, "Grasp Pose", rviz_visual_tools::WHITE, rviz_visual_tools::XSMALL, text);

    // SHOW finger_to_palm_depth
    Eigen::Vector3d grasp_point = grasp_pose.translation();
    Eigen::Vector3d obj_point = visual_tools_->convertPose(cuboid_pose_).translation();
    Eigen::Vector3d palm_vector = obj_point - grasp_point;
    palm_vector.normalize();

    Eigen::Vector3d palm_point = grasp_point + palm_vector * grasp_data_->finger_to_palm_depth_;
    visual_tools_->publishLine(grasp_point, palm_point, rviz_visual_tools::GREY);

    Eigen::Affine3d text_pose;
    Eigen::Vector3d text_point = grasp_point + palm_vector * grasp_data_->finger_to_palm_depth_ * 0.5;
    text_pose = grasp_pose;
    text_pose.translation() += text_point - grasp_pose.translation();
    visual_tools_->publishText(text_pose, "finger_to_palm_depth", rviz_visual_tools::GREY, rviz_visual_tools::XSMALL,
                               text);

    // SHOW PRE_GRASP_APPROACH
    Eigen::Vector3d pregrasp_vector =
        Eigen::Vector3d(grasp_candidates_[50]->grasp_.pre_grasp_approach.direction.vector.x,
                        grasp_candidates_[50]->grasp_.pre_grasp_approach.direction.vector.y,
                        grasp_candidates_[50]->grasp_.pre_grasp_approach.direction.vector.z);
    pregrasp_vector.normalize();
    Eigen::Vector3d approach_point =
        ee_pose.translation() +
        ee_pose.rotation() * pregrasp_vector * grasp_candidates_[50]->grasp_.pre_grasp_approach.desired_distance;
    display_pose = ee_pose;
    display_pose.translation() += approach_point - ee_pose.translation();
    visual_tools_->publishSphere(approach_point, rviz_visual_tools::PURPLE, 0.02);
    visual_tools_->publishText(display_pose, "Pre-grasp desired", rviz_visual_tools::WHITE, rviz_visual_tools::XSMALL,
                               text);

    Eigen::Vector3d approach_point_min =
        ee_pose.translation() +
        ee_pose.rotation() * pregrasp_vector * grasp_candidates_[50]->grasp_.pre_grasp_approach.min_distance;
    display_pose = ee_pose;
    display_pose.translation() += approach_point_min - ee_pose.translation();
    visual_tools_->publishSphere(approach_point_min, rviz_visual_tools::PINK, 0.02);
    visual_tools_->publishText(display_pose, "Pre-grasp min", rviz_visual_tools::WHITE, rviz_visual_tools::XSMALL,
                               text);

    visual_tools_->publishLine(approach_point, approach_point_min, rviz_visual_tools::GREY);

    // SHOW POST_GRASP_RETREAT
    Eigen::Vector3d postgrasp_vector =
        Eigen::Vector3d(grasp_candidates_[50]->grasp_.post_grasp_retreat.direction.vector.x,
                        grasp_candidates_[50]->grasp_.post_grasp_retreat.direction.vector.y,
                        grasp_candidates_[50]->grasp_.post_grasp_retreat.direction.vector.z);
    postgrasp_vector.normalize();
    Eigen::Vector3d retreat_point =
        ee_pose.translation() -
        ee_pose.rotation() * postgrasp_vector * grasp_candidates_[50]->grasp_.post_grasp_retreat.desired_distance;
    Eigen::Affine3d block_pose;
    block_pose = ee_pose;
    block_pose.translation() += retreat_point - ee_pose.translation();
    visual_tools_->publishCuboid(block_pose, 0.015, 0.015, 0.015, rviz_visual_tools::ORANGE);
    visual_tools_->publishText(block_pose, "Post-grasp desired", rviz_visual_tools::DARK_GREY,
                               rviz_visual_tools::XSMALL, text);

    Eigen::Vector3d retreat_point_min =
        ee_pose.translation() -
        ee_pose.rotation() * postgrasp_vector * grasp_candidates_[50]->grasp_.post_grasp_retreat.min_distance;
    block_pose = ee_pose;
    block_pose.translation() += retreat_point_min - ee_pose.translation();
    visual_tools_->publishCuboid(block_pose, 0.015, 0.015, 0.015, rviz_visual_tools::YELLOW);
    visual_tools_->publishText(block_pose, "Post-grasp min", rviz_visual_tools::DARK_GREY, rviz_visual_tools::XSMALL,
                               text);

    visual_tools_->publishLine(retreat_point, retreat_point_min, rviz_visual_tools::DARK_GREY);

    // SHOW ROBOT GRIPPER
    std::vector<GraspCandidatePtr> visualized_grasp;
    visualized_grasp.push_back(grasp_candidates_[50]);
    ROS_WARN_STREAM_NAMED("grasp_poses_visualizer", "TODO enable this");
    // visual_tools_->publishGrasps(visualized_grasp, grasp_data_->ee_jmg_);
  }

  void generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& l, double& w, double& h)
  {
    // Size
    l = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    w = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    h = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    ROS_INFO_STREAM_NAMED("random_cuboid", "Size = " << l << ", " << w << ", " << h);

    // Position
    rviz_visual_tools::RandomPoseBounds pose_bounds(CUBOID_WORKSPACE_MIN_X, CUBOID_WORKSPACE_MAX_X,
                                                    CUBOID_WORKSPACE_MIN_Y, CUBOID_WORKSPACE_MAX_Y,
                                                    CUBOID_WORKSPACE_MIN_Z, CUBOID_WORKSPACE_MAX_Z);
    // Orientation
    visual_tools_->generateRandomPose(cuboid_pose, pose_bounds);

    ROS_INFO_STREAM_NAMED("random_cuboid", "Position = " << cuboid_pose.position.x << ", " << cuboid_pose.position.y
                                                         << ", " << cuboid_pose.position.z);
    ROS_INFO_STREAM_NAMED("random_cuboid", "Quaternion = " << cuboid_pose.orientation.x << ", "
                                                           << cuboid_pose.orientation.y << ", "
                                                           << cuboid_pose.orientation.z);
  }

  double fRand(double fMin, double fMax)
  {
    return fMin + ((double)rand() / RAND_MAX) * (fMax - fMin);
  }

};  // class

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_poses_visualizer");

  ROS_INFO_STREAM_NAMED("main", "Grasp Poses Visualizer");

  bool verbose = false;

  moveit_grasps::GraspPosesVisualizer visualizer(verbose);

  return 0;
}
