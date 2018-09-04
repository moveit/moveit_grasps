#include "moveit_grasps/grasp_plan.h"

namespace moveit_grasps {
GraspPlanner::GraspPlanner(ros::NodeHandle nh) : nh_(nh) {
  grasp_planning_service_ = nh_.advertiseService(
      "/plan_grasps", &GraspPlanner::planGraspSrvCallback, this);
  ROS_INFO_NAMED("moveit_graps", "Ready to call service.");
  rosparam_shortcuts::get("moveit_grasps", nh_, "moveit_grasps/planner/verbose",
                          verbose_);
}

bool GraspPlanner::planGraspSrvCallback(
    moveit_msgs::GraspPlanning::Request &req,
    moveit_msgs::GraspPlanning::Response &res) {
  // Check if candidate grasps are set
  if (grasp_candidates_.empty()) {
    ROS_INFO_STREAM_NAMED("moviet_grasps/grasp_plan",
                          "grasp candidates not set, please call "
                          "set_candidate_grasps() function to set it.");
    res.error_code.val = res.error_code.FAILURE;
    return false;
  }

  for (auto each : grasp_candidates_) {
    if (verbose_) {
      ROS_INFO_STREAM_NAMED("moviet_grasps/grasp_plan", each->grasp_);
      verbose_ = false;
    }
    res.grasps.push_back(each->grasp_);
  }
  res.error_code.val = res.error_code.SUCCESS;
  std::cout << "Service is working" << std::endl;

  return true;
}

void GraspPlanner::setCandidateGrasps(
    std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates) {
  grasp_candidates_ = grasp_candidates;
}
}