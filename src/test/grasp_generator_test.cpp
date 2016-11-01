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

/* Author: Dave Coleman
   Desc:   Tests the grasp generator
*/

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Grasp generation
#include <moveit_grasps/grasp_generator.h>

namespace moveit_grasps
{
static const double BLOCK_SIZE = 0.04;

class GraspGeneratorTest
{
private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp generator
  moveit_grasps::GraspGeneratorPtr grasp_generator_;

  // class for publishing stuff to rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  rviz_visual_tools::RvizVisualToolsPtr grasp_visuals_;

  // robot-specific data for generating grasps
  moveit_grasps::GraspDataPtr grasp_data_;

  // which baxter arm are we using
  std::string ee_group_name_;
  std::string planning_group_name_;

public:
  // Constructor
  GraspGeneratorTest(int num_tests) : nh_("~")
  {
    nh_.param("ee_group_name", ee_group_name_, std::string("left_hand"));

    ROS_INFO_STREAM_NAMED("test", "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("test", "Planning Group: " << planning_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load the Robot Viz Tools for publishing to Rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base"));
    visual_tools_->deleteAllMarkers();

    grasp_visuals_.reset(new rviz_visual_tools::RvizVisualTools("base", "grasp_visuals"));
    grasp_visuals_->deleteAllMarkers();

    // ---------------------------------------------------------------------------------------------
    // Load grasp data specific to our robot
    grasp_data_.reset(new moveit_grasps::GraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()));

    const moveit::core::JointModelGroup* ee_jmg = visual_tools_->getRobotModel()->getJointModelGroup(ee_group_name_);

    // ---------------------------------------------------------------------------------------------
    // Load grasp generator
    grasp_generator_.reset(new moveit_grasps::GraspGenerator(visual_tools_, true));
    grasp_generator_->setVerbose(true);

    grasp_generator_->ideal_grasp_pose_ = Eigen::Affine3d::Identity();
    grasp_generator_->ideal_grasp_pose_ = grasp_generator_->ideal_grasp_pose_ *
                                          Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()) *
                                          Eigen::AngleAxisd(-M_PI / 2.0, Eigen::Vector3d::UnitX());
    grasp_generator_->ideal_grasp_pose_.translation() = Eigen::Vector3d(0, 0, 0.5);

    // Visualize poses
    grasp_visuals_->publishAxisLabeled(grasp_generator_->ideal_grasp_pose_, "IDEAL_EE_GRASP_POSE");

    Eigen::Affine3d grasp_to_eef = grasp_generator_->ideal_grasp_pose_ * grasp_data_->grasp_pose_to_eef_pose_;
    grasp_visuals_->publishAxisLabeled(grasp_to_eef, "GRASP_POSE_TO_EEF_POSE_");

    // publish world coordinate system
    grasp_visuals_->publishAxis(Eigen::Affine3d::Identity());

    geometry_msgs::Pose pose;
    visual_tools_->generateEmptyPose(pose);

    // ---------------------------------------------------------------------------------------------
    // Animate open and closing end effector
    if (false)
    {
      for (std::size_t i = 0; i < 4; ++i)
      {
        // Test visualization of end effector in OPEN position
        grasp_data_->setRobotStatePreGrasp(visual_tools_->getSharedRobotState());
        visual_tools_->publishEEMarkers(pose, ee_jmg, rviz_visual_tools::ORANGE, "test_eef");
        ros::Duration(1.0).sleep();

        // Test visualization of end effector in CLOSED position
        grasp_data_->setRobotStateGrasp(visual_tools_->getSharedRobotState());
        visual_tools_->publishEEMarkers(pose, ee_jmg, rviz_visual_tools::GREEN, "test_eef");
        ros::Duration(1.0).sleep();
      }
    }

    // ---------------------------------------------------------------------------------------------
    // Generate grasps for a bunch of random objects
    geometry_msgs::Pose object_pose;
    std::vector<moveit_grasps::GraspCandidatePtr> possible_grasps;

    // Loop
    int i = 0;
    while (ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test", "Adding random object " << i + 1 << " of " << num_tests);

      // Remove randomness when we are only running one test
      if (num_tests == 1)
        generateTestObject(object_pose);
      else
        generateRandomObject(object_pose);

      // Show the block
      // visual_tools_->publishBlock(object_pose, rviz_visual_tools::BLUE, BLOCK_SIZE);

      possible_grasps.clear();

      // Generate set of grasps for one object
      double depth = 0.15;
      double width = 0.05;
      double height = 0.15;

      grasp_visuals_->publishCuboid(object_pose, depth, width, height, rviz_visual_tools::TRANSLUCENT_DARK);
      grasp_visuals_->publishAxis(object_pose, rviz_visual_tools::MEDIUM);

      grasp_generator_->generateGrasps(visual_tools_->convertPose(object_pose), depth, width, height, grasp_data_,
                                       possible_grasps);

      // Visualize them
      // visual_tools_->publishAnimatedGrasps(possible_grasps, ee_jmg);
      // double animate_speed = 0.1;
      // visual_tools_->publishGrasps(possible_grasps, ee_jmg, animate_speed);

      // Test if done
      ++i;
      if (i >= num_tests)
        break;
    }
  }

  void generateTestObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    geometry_msgs::Pose start_object_pose;

    start_object_pose.position.x = 0.5;
    start_object_pose.position.y = 0.0;
    start_object_pose.position.z = 0.5;

    // Orientation
    double angle = M_PI / 1.5;
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    start_object_pose.orientation.x = quat.x();
    start_object_pose.orientation.y = quat.y();
    start_object_pose.orientation.z = quat.z();
    start_object_pose.orientation.w = quat.w();

    // Choose which object to test
    object_pose = start_object_pose;
    // visual_tools_->publishObject( object_pose, OBJECT_SIZE, true );
  }

  void generateRandomObject(geometry_msgs::Pose& object_pose)
  {
    // Position
    object_pose.position.x = fRand(0.1, 0.9);  // 0.55);
    object_pose.position.y = fRand(-0.28, 0.28);
    object_pose.position.z = 0.02;

    // Orientation
    double angle = M_PI * fRand(0.1, 1);
    Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
    object_pose.orientation.x = quat.x();
    object_pose.orientation.y = quat.y();
    object_pose.orientation.z = quat.z();
    object_pose.orientation.w = quat.w();
  }

  double fRand(double fMin, double fMax)
  {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  }

};  // end of class

}  // namespace

int main(int argc, char* argv[])
{
  int num_tests = 1;
  ros::init(argc, argv, "grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main", "GraspGenerator Test");

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Seed random
  srand(ros::Time::now().toSec());

  // Benchmark time
  ros::Time start_time;
  start_time = ros::Time::now();

  // Run Tests
  moveit_grasps::GraspGeneratorTest tester(num_tests);

  // Benchmark time
  double duration = (ros::Time::now() - start_time).toNSec() * 1e-6;
  ROS_INFO_STREAM_NAMED("", "Total time: " << duration);
  // std::cout << duration << "\t" << num_tests << std::endl;

  return 0;
}
