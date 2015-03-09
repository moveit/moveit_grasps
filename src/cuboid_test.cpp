/* Author: Andy McEvoy
   Desc:   Tests the cuboid grasp generator
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

class CuboidGraspGeneratorTest
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
  CuboidGraspGeneratorTest(size_t number_of_trials, bool verbose) : nh_("~")
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
    grasps_.reset( new moveit_grasps::Grasps(visual_tools_, verbose) );

    // initialize cuboid size
    depth_ = CUBOID_MIN_SIZE;
    width_ = CUBOID_MIN_SIZE;
    height_ = CUBOID_MIN_SIZE;
    max_grasp_size_ = MODEL_T_MAX_GRASP_SIZE;
    // Seed random
    srand(ros::Time::now().toSec());

    visual_tools_->deleteAllMarkers();

    int completed_trials = 0;
    while(ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test","\n************* \nStarting test " 
                            << completed_trials + 1 << " of " << number_of_trials << "\n*************");

      ROS_INFO_STREAM_NAMED("test", "generating random cuboid");
      generateRandomCuboid(cuboid_pose_,depth_,width_,height_);

      visual_tools_->publishRectangle(cuboid_pose_,depth_,width_,height_);
      visual_tools_->publishAxis(cuboid_pose_);

      ROS_INFO_STREAM_NAMED("test","generating grasps for cuboid");
    
      possible_grasps_.clear();

      grasps_->generateCuboidGrasps( visual_tools_->convertPose(cuboid_pose_), depth_, width_, height_, 
                                     max_grasp_size_, grasp_data_, possible_grasps_);
      ROS_INFO_STREAM_NAMED("test", "Generated " << possible_grasps_.size() << " possible grasps around cuboid ");

      //visual_tools_->publishAnimatedGrasps(possible_grasps_, ee_jmg_);
      visual_tools_->publishGrasps(possible_grasps_, ee_jmg_);

      completed_trials++;
      if (completed_trials == number_of_trials)
        break;
    }
  }

  void animateGripperOpenClose(int number_of_trials) 
  {
    geometry_msgs::Pose pose;
    visual_tools_->generateEmptyPose(pose);

    // TODO: visualization doesn't work... need to investigate
    for (int i = 0; i < number_of_trials; i++) 
      {
	// open position
	ROS_DEBUG_STREAM_NAMED("animate", "animating OPEN gripper");
	grasp_data_.setRobotStatePreGrasp( visual_tools_->getSharedRobotState() );
	visual_tools_->publishEEMarkers(pose, ee_jmg_, rviz_visual_tools::ORANGE, "test_eef");
	ros::Duration(1.0).sleep();
	
	// close position
	ROS_DEBUG_STREAM_NAMED("animate", "animating CLOSE gripper");
	grasp_data_.setRobotStateGrasp( visual_tools_->getSharedRobotState() );
	visual_tools_->publishEEMarkers(pose, ee_jmg_, rviz_visual_tools::GREEN, "test_eef");
	ros::Duration(1.0).sleep();
      }
  }

  void generateRandomCuboid(geometry_msgs::Pose& cuboid_pose, double& l, double& w, double& h)
  {
    // Size
    l = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    w = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    h = fRand(CUBOID_MIN_SIZE, CUBOID_MAX_SIZE);
    ROS_DEBUG_STREAM_NAMED("random_cuboid","Size = " << l << ", "<< w << ", " << h);

    // Position
    // Values chosen to be within shelf boundary for Amazon pick & place challenge
    // TODO: get right values
    visual_tools_->random_pose_bounds_.x_min_ = CUBOID_WORKSPACE_MIN_X; 
    visual_tools_->random_pose_bounds_.x_max_ =  CUBOID_WORKSPACE_MAX_X;

    visual_tools_->random_pose_bounds_.y_min_ = CUBOID_WORKSPACE_MIN_Y; 
    visual_tools_->random_pose_bounds_.y_max_ =  CUBOID_WORKSPACE_MAX_Y;

    visual_tools_->random_pose_bounds_.z_min_ = CUBOID_WORKSPACE_MIN_Z; 
    visual_tools_->random_pose_bounds_.z_max_ =  CUBOID_WORKSPACE_MAX_Z;

    // Orientation 
    visual_tools_->generateRandomPose(cuboid_pose);

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
  ros::init(argc, argv, "cuboid_grasp_generator_test");

  ROS_INFO_STREAM_NAMED("main","Cuboid Grasp Tests");
  
  int number_of_trials = 1;
  bool verbose = false;

  if (argc > 1) 
  {
    for (std::size_t i = 0; i < argc; i++) 
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        i++;
        if (strcmp(argv[i], "true") == 0 )
        {
          ROS_INFO_STREAM_NAMED("main","Running in VERBOSE mode (slower)");
          verbose = true;
        }
        continue;
      }

      if (strcmp(argv[i], "--trials" ) == 0)
      {
        i++;
        number_of_trials = std::atoi(argv[i]);
        continue;
      }  
    }
    ROS_INFO_STREAM_NAMED("main","Running " << number_of_trials << " trials");

  }
  moveit_grasps::CuboidGraspGeneratorTest tester(number_of_trials, verbose);

  return 0;
}
