#ifndef MOVEIT_GRASPS__GRASP_PLANNER_
#define MOVEIT_GRASPS__GRASP_PLANNER_

#include "ros/ros.h"
#include "moveit_msgs/GraspPlanning.h"

// Grasp
#include <moveit_grasps/grasp_candidate.h>
// #include <moveit_grasps/grasp_filter.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// robot specific properties
// #include <moveit_grasps/grasp_data.h>

// Parameter loading
// #include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps
{
class GraspPlanner
{
public:
  GraspPlanner(ros::NodeHandle nh);
  bool planGraspSrvCallback(moveit_msgs::GraspPlanning::Request  &req, moveit_msgs::GraspPlanning::Response &res);
  void setCandidateGrasps(std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates);

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp panning service server
  ros::ServiceServer grasp_planning_service_;

  // Grasp generator
  // moveit_grasps::GraspGeneratorPtr grasp_generator_;

  // Grasp candidate
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  // data for generating grasps
  // moveit_grasps::GraspDataPtr grasp_data_;
};

typedef boost::shared_ptr<GraspPlanner> GraspPlannerPtr;
}

#endif