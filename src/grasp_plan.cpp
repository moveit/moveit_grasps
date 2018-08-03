#include "moveit_grasps/grasp_plan.h"

namespace moveit_grasps
{
GraspPlanner::GraspPlanner(ros::NodeHandle nh)
  : nh_(nh)
{
  grasp_planning_service_ = nh_.advertiseService("/plan_grasps", &GraspPlanner::planGraspSrvCallback, this);
  ROS_INFO_NAMED("moveit_graps", "Ready to call service.");
}

void openGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL open_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as open, wide enough for the object to fit. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.04;
  posture.points[0].positions[1] = 0.04;
  // END_SUB_TUTORIAL
}

void closedGripper(trajectory_msgs::JointTrajectory& posture)
{
  // BEGIN_SUB_TUTORIAL closed_gripper
  /* Add both finger joints of panda robot. */
  posture.joint_names.resize(2);
  posture.joint_names[0] = "panda_finger_joint1";
  posture.joint_names[1] = "panda_finger_joint2";

  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(2);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  // END_SUB_TUTORIAL
}

// void pick(std::vector<moveit_msgs::Grasp> grasps)
// {
//   // BEGIN_SUB_TUTORIAL pick1
//   // Create a vector of grasps to be attempted, currently only creating single grasp.
//   // This is essentially useful when using a grasp generator to generate and test multiple grasps.
//   // std::vector<moveit_msgs::Grasp> grasps;
//   grasps.resize(1);

//   // Setting grasp pose
//   // ++++++++++++++++++++++
//   // This is the pose of panda_link8. |br|
//   // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
//   // of the cube). |br|
//   // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
//   // extra padding)
//   graspss.grasp_pose.header.frame_id = "panda_link0";
//   graspss.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2, -M_PI / 4, -M_PI / 2);
//   graspss.grasp_pose.pose.position.x = 0.415;
//   graspss.grasp_pose.pose.position.y = 0;
//   graspss.grasp_pose.pose.position.z = 0.5;

//   // Setting pre-grasp approach
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   graspss.pre_grasp_approach.direction.header.frame_id = "panda_link0";
//   /* Direction is set as positive x axis */
//   graspss.pre_grasp_approach.direction.vector.x = 1.0;
//   graspss.pre_grasp_approach.min_distance = 0.095;
//   graspss.pre_grasp_approach.desired_distance = 0.115;

//   // Setting post-grasp retreat
//   // ++++++++++++++++++++++++++
//   /* Defined with respect to frame_id */
//   graspss.post_grasp_retreat.direction.header.frame_id = "panda_link0";
//   /* Direction is set as positive z axis */
//   graspss.post_grasp_retreat.direction.vector.z = 1.0;
//   graspss.post_grasp_retreat.min_distance = 0.1;
//   graspss.post_grasp_retreat.desired_distance = 0.25;

//   // Setting posture of eef before grasp
//   // +++++++++++++++++++++++++++++++++++
//   openGripper(graspss.pre_grasp_posture);
//   // END_SUB_TUTORIAL

//   // BEGIN_SUB_TUTORIAL pick2
//   // Setting posture of eef during grasp
//   // +++++++++++++++++++++++++++++++++++
//   closedGripper(graspss.grasp_posture);
//   // END_SUB_TUTORIAL

//   // BEGIN_SUB_TUTORIAL pick3
//   // Set support surface as table1.
//   // move_group.setSupportSurfaceName("table1");
//   // END_SUB_TUTORIAL
// }

bool GraspPlanner::planGraspSrvCallback(moveit_msgs::GraspPlanning::Request  &req,
                                        moveit_msgs::GraspPlanning::Response &res)
{
  // Check if candidate grasps are set
  if (grasp_candidates_.empty())
  {
    ROS_INFO_STREAM_NAMED("moviet_grasps/grasp_plan", "grasp candidates not set, please call set_candidate_grasps() function to set it.");
    res.error_code.val = res.error_code.FAILURE;
    return false;
  }
  
  moveit_msgs::Grasp graspss;

  graspss.grasp_pose.header.frame_id = "panda_link0";
  graspss.grasp_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(-M_PI / 2, -M_PI / 4, -M_PI / 2);
  graspss.grasp_pose.pose.position.x = 0.415;
  graspss.grasp_pose.pose.position.y = 0;
  graspss.grasp_pose.pose.position.z = 0.5;

  // Setting pre-grasp approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  graspss.pre_grasp_approach.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive x axis */
  graspss.pre_grasp_approach.direction.vector.x = 1.0;
  graspss.pre_grasp_approach.min_distance = 0.095;
  graspss.pre_grasp_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  graspss.post_grasp_retreat.direction.header.frame_id = "panda_link0";
  /* Direction is set as positive z axis */
  graspss.post_grasp_retreat.direction.vector.z = 1.0;
  graspss.post_grasp_retreat.min_distance = 0.1;
  graspss.post_grasp_retreat.desired_distance = 0.25;

  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(graspss.pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(graspss.grasp_posture);


  // moveit_msgs::Grasp graspss.;
  // pick(graspss.);
  res.grasps.push_back(graspss);
  for (auto each: grasp_candidates_)
  {
    res.grasps.push_back(each->grasp_);
  }
  res.error_code.val = res.error_code.SUCCESS;
  std::cout << "Service is working" << std::endl;

  return true;
}

void GraspPlanner::setCandidateGrasps(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates)
{
  grasp_candidates_ = grasp_candidates;
}
}