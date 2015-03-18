/* Author: Andy McEvoy
   Desc:   creates a vizualization of all the poses used in the graping pipeline
*/

#include <moveit_grasps/grasps.h>

namespace moveit_grasps
{

// Size and location for randomly generated cuboids
static const double CUBOID_MIN_SIZE = 0.02;
static const double CUBOID_MAX_SIZE = 0.15;
static const double CUBOID_WORKSPACE_MIN_X = 0.01; 
static const double CUBOID_WORKSPACE_MAX_X = 0.5;
static const double CUBOID_WORKSPACE_MIN_Y = -0.5;
static const double CUBOID_WORKSPACE_MAX_Y = 0.5;
static const double CUBOID_WORKSPACE_MIN_Z = 0.0;
static const double CUBOID_WORKSPACE_MAX_Z = 1.0;

// TODO: verify max object size Open Hand can grasp
static const double MODEL_T_MAX_GRASP_SIZE = 0.10;

class GraspPosesVisualizer
{

private:
  ros::NodeHandle nh_;

  // cuboid dimensions
  double depth_;
  double width_;
  double height_;
  double max_grasp_size_;
  geometry_msgs::Pose cuboid_pose_;
  moveit_grasps::GraspsPtr grasps_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  std::vector<moveit_msgs::Grasp> possible_grasps_;
  moveit_grasps::GraspData grasp_data_;
  const moveit::core::JointModelGroup* ee_jmg_;

  // TODO: read in from param

  // arm description
  std::string ee_group_name_;
  std::string planning_group_name_;

public:

  // Constructor
  GraspPosesVisualizer() : nh_("~")
  {
    // get arm parameters
    nh_.param("ee_group_name", ee_group_name_, std::string("left_hand"));

    ROS_INFO_STREAM_NAMED("init", "End Effector: " << ee_group_name_);
    ROS_INFO_STREAM_NAMED("init", "Planning Group: " << planning_group_name_);

    // set up rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/rviz_visual_tools"));
    visual_tools_->setMuted(false);
    visual_tools_->loadMarkerPub();

    // load grasp data 
    if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_, visual_tools_->getRobotModel()))
      {
    	ROS_ERROR_STREAM_NAMED("init", "Failed to load grasp data");
    	ros::shutdown();
      }
    ee_jmg_ = visual_tools_->getRobotModel()->getJointModelGroup(ee_group_name_);

    // load grasp generator
    bool verbose = false;
    grasps_.reset( new moveit_grasps::Grasps(visual_tools_, verbose) );

    // initialize cuboid size
    depth_ = CUBOID_MIN_SIZE;
    width_ = CUBOID_MIN_SIZE;
    height_ = CUBOID_MIN_SIZE;
    max_grasp_size_ = MODEL_T_MAX_GRASP_SIZE;
    // Seed random
    srand(ros::Time::now().toSec());

    visual_tools_->deleteAllMarkers();

    ROS_INFO_STREAM_NAMED("viz_test","************* \nStarting Vizualization" << "\n*************");

    ROS_INFO_STREAM_NAMED("viz_test", "generating random cuboid");
    generateRandomCuboid(cuboid_pose_,depth_,width_,height_);

    // SHOW OBJECT POSE
    visual_tools_->publishRectangle(cuboid_pose_,depth_,width_,height_);
    visual_tools_->publishAxis(cuboid_pose_);
    visual_tools_->publishText(cuboid_pose_,"Object Pose");

    ROS_INFO_STREAM_NAMED("viz_test","generating grasps for cuboid");
    possible_grasps_.clear();
    grasps_->generateCuboidGrasps( visual_tools_->convertPose(cuboid_pose_), depth_, width_, height_, 
                                     max_grasp_size_, grasp_data_, possible_grasps_);
    ROS_INFO_STREAM_NAMED("viz_test", "Generated " << possible_grasps_.size() << " possible grasps around cuboid ");

    // SHOW GRIPPER GRASP POSE
    //visual_tools_->publishGrasps(possible_grasps_[0], ee_jmg_);
    visual_tools_->publishAxis(possible_grasps_[0].grasp_pose.pose);
    visual_tools_->publishSphere(possible_grasps_[0].grasp_pose.pose, rviz_visual_tools::PINK, rviz_visual_tools::XSMALL);
    visual_tools_->publishText(possible_grasps_[0].grasp_pose.pose, "Gripper Grasp Pose");
    
  }

  void generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& l, double& w, double& h)
  {
    // Size
    l = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    w = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    h = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    ROS_INFO_STREAM_NAMED("random_cuboid","Size = " << l << ", "<< w << ", " << h);

    // Position
    rviz_visual_tools::RandomPoseBounds pose_bounds(CUBOID_WORKSPACE_MIN_X, CUBOID_WORKSPACE_MAX_X, 
                                                    CUBOID_WORKSPACE_MIN_Y, CUBOID_WORKSPACE_MAX_Y, 
                                                    CUBOID_WORKSPACE_MIN_Z, CUBOID_WORKSPACE_MAX_Z);
    // Orientation 
    visual_tools_->generateRandomPose(cuboid_pose, pose_bounds);

    ROS_INFO_STREAM_NAMED("random_cuboid","Position = " << cuboid_pose.position.x << ", " << 
    			   cuboid_pose.position.y << ", " << cuboid_pose.position.z);
    ROS_INFO_STREAM_NAMED("random_cuboid","Quaternion = " << cuboid_pose.orientation.x << ", " <<
			   cuboid_pose.orientation.y << ", " << cuboid_pose.orientation.z);
  }

  double fRand(double fMin, double fMax) 
  {
    return fMin + ( (double)rand() / RAND_MAX ) * (fMax - fMin);
  }

}; // class

} // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "grasp_poses_visualizer");

  ROS_INFO_STREAM_NAMED("main","Grasp Poses Visualizer");

  moveit_grasps::GraspPosesVisualizer visualizer();

  return 0;
}
