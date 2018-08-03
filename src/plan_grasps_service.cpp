#include "ros/ros.h"
#include "moveit_msgs/GraspPlanning.h"

bool plan_grasps(moveit_msgs::GraspPlanning::Request  &req,
                 moveit_msgs::GraspPlanning::Response &res)
{
  std::cout << "Service is working" << std::endl;
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasp_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("plan_grasps", plan_grasps);
  ROS_INFO("Ready to call service.");
  ros::spin();

  return 0;
}