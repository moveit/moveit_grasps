#ifndef MOVEIT_GRASPS__GRASP_PLANNER_
#define MOVEIT_GRASPS__GRASP_PLANNER_

#include "moveit_msgs/GraspPlanning.h"
#include "ros/ros.h"

// Grasp
#include <moveit_grasps/grasp_candidate.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

// Parameter loading
#include <rosparam_shortcuts/rosparam_shortcuts.h>

namespace moveit_grasps {
/**
* \brief Impliments the grasp_plan service which is called when calling the
* function
*        planGraspsAndPick().
*/
class GraspPlanner {
public:
  GraspPlanner(ros::NodeHandle nh);
  bool planGraspSrvCallback(moveit_msgs::GraspPlanning::Request &req,
                            moveit_msgs::GraspPlanning::Response &res);
  void setCandidateGrasps(
      std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates);

private:
  // A shared node handle
  ros::NodeHandle nh_;

  // Grasp panning service server
  ros::ServiceServer grasp_planning_service_;

  // Grasp candidate
  std::vector<moveit_grasps::GraspCandidatePtr> grasp_candidates_;

  // Tool for visualizing things in Rviz
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

  bool verbose_;
};

typedef boost::shared_ptr<GraspPlanner> GraspPlannerPtr;
}

#endif