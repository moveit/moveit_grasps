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
   Desc:   Creates a vizualization of all the poses used in the grasping pipeline
*/

#include <rviz_visual_tools/rviz_visual_tools.h>

#include <moveit_grasps/two_finger_grasp_generator.h>
#include <moveit_grasps/two_finger_grasp_data.h>
#include <string>
#include <vector>

namespace moveit_grasps
{
// Size and location for randomly generated cuboids
static const double CUBOID_MIN_SIZE = 0.01;
static const double CUBOID_MAX_SIZE = 0.02;
static const double CUBOID_WORKSPACE_MIN_X = 0.3;
static const double CUBOID_WORKSPACE_MAX_X = 0.5;
static const double CUBOID_WORKSPACE_MIN_Y = -0.125;
static const double CUBOID_WORKSPACE_MAX_Y = 0.125;
static const double CUBOID_WORKSPACE_MIN_Z = 0.3;
static const double CUBOID_WORKSPACE_MAX_Z = 0.6;

class GraspPosesVisualizer
{
public:
  // Constructor
  explicit GraspPosesVisualizer(bool verbose, const std::string name) : nh_("~"), name_(name)
  {
    // get arm parameters
    nh_.param("ee_group_name", ee_group_name_, std::string("hand"));
    nh_.param("planning_group_name", planning_group_name_, std::string("panda_arm"));

    ROS_INFO_STREAM_NAMED("init", "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("init", "Planning Group: " << planning_group_name_);

    // set up rviz
    visual_tools_ = std::make_shared<moveit_visual_tools::MoveItVisualTools>("world", "/rviz_visual_tools");
    visual_tools_->loadMarkerPub();
    visual_tools_->loadRobotStatePub("/display_robot_state");
    visual_tools_->loadTrajectoryPub("/display_planned_path");
    visual_tools_->loadSharedRobotState();
    visual_tools_->getSharedRobotState()->setToDefaultValues();
    visual_tools_->enableBatchPublishing();
    visual_tools_->deleteAllMarkers();
    visual_tools_->removeAllCollisionObjects();
    visual_tools_->hideRobot();
    visual_tools_->trigger();

    const moveit::core::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(ee_group_name_);

    // Load grasp data
    grasp_data_ = std::make_shared<TwoFingerGraspData>(nh_, ee_group_name_, visual_tools_->getRobotModel());
    ROS_ASSERT_MSG(grasp_data_->loadGraspData(nh_, ee_group_name_), "Failed to load Grasp Data parameters.");

    // load grasp generator
    grasp_generator_ = std::make_shared<moveit_grasps::TwoFingerGraspGenerator>(visual_tools_, verbose);

    // initialize cuboid size
    depth_ = CUBOID_MIN_SIZE;
    width_ = CUBOID_MIN_SIZE;
    height_ = CUBOID_MIN_SIZE;

    // Seed random
    srand(ros::Time::now().toSec());

    ROS_INFO_STREAM_NAMED(name_, "\n************* \nStarting Vizualization"
                                     << "\n*************");

    ROS_INFO_STREAM_NAMED(name_, "generating random cuboid");
    generateRandomCuboid(cuboid_pose_, depth_, width_, height_);

    Eigen::Isometry3d display_pose;
    bool text = false;
    rviz_visual_tools::scales text_size = rviz_visual_tools::MEDIUM;

    // SHOW OBJECT POSE
    ROS_INFO_STREAM_NAMED(name_, "Publishing random cube");
    visual_tools_->publishCuboid(cuboid_pose_, depth_, width_, height_, rviz_visual_tools::TRANSLUCENT_DARK);
    visual_tools_->publishAxis(cuboid_pose_, 0.05, 0.005);
    geometry_msgs::Pose cuboid_text_pose(cuboid_pose_);
    cuboid_text_pose.position.z += 0.05;
    visual_tools_->publishText(cuboid_text_pose, "Object Pose", rviz_visual_tools::WHITE, text_size, text);
    visual_tools_->trigger();

    ROS_INFO_STREAM_NAMED(name_, "Generating grasps");

    grasp_candidates_.clear();
    moveit_grasps::TwoFingerGraspCandidateConfig grasp_generator_config =
        moveit_grasps::TwoFingerGraspCandidateConfig();
    grasp_generator_config.disableAll();
    grasp_generator_config.enable_face_grasps_ = true;
    grasp_generator_config.generate_y_axis_grasps_ = true;
    grasp_generator_->setGraspCandidateConfig(grasp_generator_config);
    grasp_generator_->generateGrasps(visual_tools_->convertPose(cuboid_pose_), depth_, width_, height_, grasp_data_,
                                     grasp_candidates_);

    // SHOW GRASP POSE
    visual_tools_->prompt("Press 'next' to show an example eef and grasp pose");
    ROS_INFO_STREAM_NAMED(name_, "Showing the grasp pose");
    Eigen::Isometry3d eef_mount_grasp_pose =
        visual_tools_->convertPose(grasp_candidates_.front()->grasp_.grasp_pose.pose);
    visual_tools_->publishAxis(eef_mount_grasp_pose, 0.05, 0.005);
    Eigen::Isometry3d grasp_text_pose(eef_mount_grasp_pose);
    grasp_text_pose.translation().z() += 0.03;
    visual_tools_->publishText(grasp_text_pose, "Grasp Pose", rviz_visual_tools::WHITE, text_size, text);
    visual_tools_->publishSphere(eef_mount_grasp_pose.translation(), rviz_visual_tools::LIME_GREEN, 0.01);
    visual_tools_->trigger();

    // SHOW EE GRASP POSE
    ROS_INFO_STREAM_NAMED(name_, "Showing tcp grasp pose");
    Eigen::Isometry3d tcp_grasp_pose = eef_mount_grasp_pose * grasp_data_->tcp_to_eef_mount_.inverse();
    visual_tools_->publishAxis(tcp_grasp_pose, 0.05, 0.005);
    Eigen::Isometry3d tcp_text_pose(tcp_grasp_pose);
    tcp_text_pose.translation().z() += 0.03;
    visual_tools_->publishText(tcp_text_pose, "TCP Pose", rviz_visual_tools::WHITE, text_size, text);
    visual_tools_->publishSphere(tcp_grasp_pose.translation(), rviz_visual_tools::GREEN, 0.01);
    visual_tools_->trigger();

    visual_tools_->prompt("Press 'next' to visualize the grasp max and min depth");

    // SHOW grasp_max_depth
    ROS_INFO_STREAM_NAMED(name_, "Showing grasp_max_depth");
    Eigen::Vector3d palm_vector = -tcp_grasp_pose.translation() + eef_mount_grasp_pose.translation();
    palm_vector.normalize();
    Eigen::Vector3d max_grasp_depth_point =
        tcp_grasp_pose.translation() + palm_vector * (grasp_data_->grasp_max_depth_ - grasp_data_->grasp_min_depth_);
    Eigen::Vector3d min_grasp_depth_point = tcp_grasp_pose.translation();
    visual_tools_->publishLine(min_grasp_depth_point, max_grasp_depth_point, rviz_visual_tools::GREY);
    Eigen::Isometry3d min_depth_eef_pose = eef_mount_grasp_pose;
    visual_tools_->publishEEMarkers(min_depth_eef_pose, ee_jmg, grasp_data_->pre_grasp_posture_.points[0].positions,
                                    rviz_visual_tools::TRANSLUCENT_DARK, "test_eef");
    visual_tools_->trigger();

    visual_tools_->prompt("Press 'next' to visualize the pre-grasp, grasp, lift, and retreat poses");

    EigenSTL::vector_Isometry3d grasp_waypoints;
    GraspGenerator::getGraspWaypoints(grasp_candidates_.front(), grasp_waypoints);
    visual_tools_->publishAxisLabeled(grasp_waypoints[0], "pregrasp", rviz_visual_tools::SMALL);
    visual_tools_->publishAxisLabeled(grasp_waypoints[1], "grasp", rviz_visual_tools::SMALL);
    visual_tools_->publishAxisLabeled(grasp_waypoints[2], "lifted", rviz_visual_tools::SMALL);
    visual_tools_->publishAxisLabeled(grasp_waypoints[3], "retreat", rviz_visual_tools::SMALL);
    visual_tools_->trigger();
    ros::Duration(0.5).sleep();
  }

  void generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& l, double& w, double& h)
  {
    // Size
    l = rviz_visual_tools::RvizVisualTools::dRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    w = rviz_visual_tools::RvizVisualTools::dRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    h = rviz_visual_tools::RvizVisualTools::dRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
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

private:
  ros::NodeHandle nh_;
  std::string name_;

  // cuboid dimensions
  double depth_;
  double width_;
  double height_;
  geometry_msgs::Pose cuboid_pose_;
  moveit_grasps::TwoFingerGraspGeneratorPtr grasp_generator_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  std::vector<GraspCandidatePtr> grasp_candidates_;
  moveit_grasps::TwoFingerGraspDataPtr grasp_data_;

  // TODO(mcevoyandy): read in from param

  // arm description
  std::string ee_group_name_;
  std::string planning_group_name_;
};  // class

}  // namespace moveit_grasps

int main(int argc, char** argv)
{
  const std::string name = "grasp_poses_visualizer_demo";
  ros::init(argc, argv, name);

  ROS_INFO_STREAM_NAMED(name, "Grasp Poses Visualizer");

  bool verbose = false;

  moveit_grasps::GraspPosesVisualizer visualizer(verbose, name);

  return 0;
}
