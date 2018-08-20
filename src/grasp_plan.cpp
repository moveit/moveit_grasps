#include "moveit_grasps/grasp_plan.h"

namespace moveit_grasps
{
class MoveItPrinter
{
public:
  MoveItPrinter(moveit_msgs::Grasp grasp) : grasp_(grasp)
  {}

  MoveItPrinter(){}

  void id()
  {
    // printing id
    std::cout << "Grasp Id: " << grasp_.id << std::endl;
  }

  void jointTrajectory(trajectory_msgs::JointTrajectory joint_traj)
  {
    // printing joint names and their positions
    for (std::size_t i = 0; i < joint_traj.joint_names.size(); i++)
    // for (auto each: joint_traj.joint_names)
    {
      std::cout << "Joint name: " << joint_traj.joint_names[i] << std::endl;
      std::cout << "Position Values:-" << std::endl;
      // std::cout << "Joint points size: " << joint_traj.points.size() << std::endl;
      for (auto each: joint_traj.points[0].positions)
      {
        std::cout << each << "\t";
      }
      std::cout << std::endl;
    }
    std::cout<<std::endl;
  }

  void preGraspPosture()
  {
    std::cout << "Printing Pre Grasp Posture" << std::endl;
    jointTrajectory(grasp_.pre_grasp_posture);
  }

  void graspPosture()
  {
    std::cout << "Printing Grasp Posture" << std::endl;
    jointTrajectory(grasp_.grasp_posture);
  }

  void point(geometry_msgs::Point point)
  {
    std::cout << "Point: " << std::endl;
    std::cout << "    x: " << point.x << std::endl;
    std::cout << "    y: " << point.y << std::endl;
    std::cout << "    z: " << point.z << std::endl;
  }

  void quaternion(geometry_msgs::Quaternion quat)
  {
    std::cout << "Quaternion: " << std::endl;
    std::cout << "    x: " << quat.x << std::endl;
    std::cout << "    y: " << quat.y << std::endl;
    std::cout << "    z: " << quat.z << std::endl;
    std::cout << "    w: " << quat.w << std::endl;
  }

  void pose(geometry_msgs::Pose pose)
  {
    std::cout << "Pose: " << std::endl;
    point(pose.position);
    quaternion(pose.orientation);
  }

  void poseStamped(geometry_msgs::PoseStamped pose_stamped)
  {
    pose(pose_stamped.pose);
  }

  void graspPose()
  {
    poseStamped(grasp_.grasp_pose);
  }

  void graspQuality()
  {
    std::cout << "Grasp Quality: " << grasp_.grasp_quality << std::endl;
  }

  void vector3(geometry_msgs::Vector3 vect)
  {
    std::cout << "vector: " <<std::endl;
    std::cout << "   x: " << vect.x << std::endl;
    std::cout << "   y: " << vect.y << std::endl;
    std::cout << "   z: " << vect.z << std::endl;
  }

  void vector3Stamped(geometry_msgs::Vector3Stamped vect_stamped)
  {
    std::cout << "frame_id: " << vect_stamped.header.frame_id << std::endl;
    vector3(vect_stamped.vector);
  }

  void gripperTranslation(moveit_msgs::GripperTranslation grip_trans)
  {
    std::cout << "GripperTranslation: " << std::endl;
    vector3Stamped(grip_trans.direction);
    std::cout << "desired Distance: " << grip_trans.desired_distance << std::endl;
    std::cout << "Min Distance: " << grip_trans.min_distance << std::endl;
  }

  void preGraspApproach()
  {
    std::cout << "Pre Grasp Approach: " << std::endl;
    gripperTranslation(grasp_.pre_grasp_approach);
  }

  void postGraspRetreat()
  {
    std::cout << "Post Grasp Retreat: " << std::endl;
    gripperTranslation(grasp_.post_grasp_retreat);
  }

  void postPlaceRetreat()
  {
    std::cout << "Post Place Retreat: " << std::endl;
    gripperTranslation(grasp_.post_place_retreat);
  }

  void printGrasp()
  {
    id();
    preGraspPosture();
    graspPosture();
    graspPose();
    graspQuality();
    preGraspApproach();
    postGraspRetreat();
    postPlaceRetreat();
  }

  void setGrasp(moveit_msgs::Grasp grasp)
  {
    grasp_ = grasp;
  }
private:
  moveit_msgs::Grasp grasp_;
};

GraspPlanner::GraspPlanner(ros::NodeHandle nh)
  : nh_(nh)
{
  grasp_planning_service_ = nh_.advertiseService("/plan_grasps", &GraspPlanner::planGraspSrvCallback, this);
  ROS_INFO_NAMED("moveit_graps", "Ready to call service.");
  rosparam_shortcuts::get("moveit_grasps", nh_, "planner/verbose", verbose_);
}

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

  MoveItPrinter printer_obj;
  for (auto each: grasp_candidates_)
  {
    if (verbose_)
    {
      printer_obj.setGrasp(each->grasp_);
      printer_obj.printGrasp();
      verbose_ = false;
    }
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