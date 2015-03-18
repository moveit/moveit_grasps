/* Author: Andy McEvoy
   Desc:   creates a vizualization of all the poses used in the graping pipeline
*/

#include <moveit_grasps/grasps.h>

namespace moveit_grasps
{

// Size and location for randomly generated cuboids
static const double CUBOID_MIN_SIZE = 0.02;
static const double CUBOID_MAX_SIZE = 0.07;
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
  GraspPosesVisualizer(bool verbose) : nh_("~")
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

    ROS_INFO_STREAM_NAMED("viz_test","************* \nStarting Vizualization" << "\n*************");

    ROS_INFO_STREAM_NAMED("viz_test", "generating random cuboid");
    generateRandomCuboid(cuboid_pose_,depth_,width_,height_);

    Eigen::Affine3d display_pose;
    bool text = false;

    // SHOW OBJECT POSE
    visual_tools_->publishRectangle(cuboid_pose_,depth_,width_,height_);
    visual_tools_->publishAxis(cuboid_pose_, 0.05, 0.005);
    visual_tools_->publishText(cuboid_pose_,"Object Pose", rviz_visual_tools::WHITE, rviz_visual_tools::SMALL, text);
    possible_grasps_.clear();
    grasps_->generateCuboidGrasps( visual_tools_->convertPose(cuboid_pose_), depth_, width_, height_, 
                                     max_grasp_size_, grasp_data_, possible_grasps_);    

    // SHOW EE GRASP POSE (grasp_pose * grasp_data.grasp_pose_to_eef_pose_)
    display_pose = visual_tools_->convertPose(possible_grasps_[50].grasp_pose.pose);
    visual_tools_->publishAxis(display_pose, 0.05, 0.005);
    visual_tools_->publishSphere(display_pose.translation(), rviz_visual_tools::GREEN, 0.02);
    visual_tools_->publishText(display_pose, "EE Pose", rviz_visual_tools::WHITE, rviz_visual_tools::SMALL, text);

    // SHOW PRE_GRASP_APPROACH
    Eigen::Vector3d pregrasp_vector = Eigen::Vector3d(possible_grasps_[50].pre_grasp_approach.direction.vector.x,
                                                      possible_grasps_[50].pre_grasp_approach.direction.vector.y,
                                                      possible_grasps_[50].pre_grasp_approach.direction.vector.z);
    ROS_DEBUG_STREAM_NAMED("viz_test","pregrasp_vector\n" << pregrasp_vector);
    double pregrasp_distance = possible_grasps_[50].pre_grasp_approach.desired_distance;
    double pregrasp_distance_min = possible_grasps_[50].pre_grasp_approach.min_distance;
    Eigen::Vector3d pregrasp_point = display_pose.translation() - pregrasp_vector * pregrasp_distance;
    Eigen::Vector3d pregrasp_min_point = display_pose.translation() - pregrasp_vector * pregrasp_distance_min;
    visual_tools_->publishLine(pregrasp_point,pregrasp_min_point);
    visual_tools_->publishSphere(pregrasp_point, rviz_visual_tools::PURPLE, 0.02);
    visual_tools_->publishSphere(pregrasp_min_point, rviz_visual_tools::PINK,0.02);
    
    // SHOW POST_GRASP_RETREAT
    Eigen::Vector3d postgrasp_vector = Eigen::Vector3d(possible_grasps_[50].post_grasp_retreat.direction.vector.x,
                                                       possible_grasps_[50].post_grasp_retreat.direction.vector.y,
                                                       possible_grasps_[50].post_grasp_retreat.direction.vector.z);
    ROS_DEBUG_STREAM_NAMED("viz_test","postgrasp_vector\n" << postgrasp_vector);
    double postgrasp_distance = possible_grasps_[50].post_grasp_retreat.desired_distance;
    double postgrasp_distance_min = possible_grasps_[50].post_grasp_retreat.min_distance;
    Eigen::Vector3d postgrasp_point = display_pose.translation() + postgrasp_vector * postgrasp_distance;
    Eigen::Vector3d postgrasp_min_point = display_pose.translation() + postgrasp_vector * postgrasp_distance_min;
    visual_tools_->publishLine(postgrasp_point,postgrasp_min_point);
    visual_tools_->publishSphere(postgrasp_point, rviz_visual_tools::ORANGE, 0.02);
    visual_tools_->publishSphere(postgrasp_min_point, rviz_visual_tools::YELLOW,0.02);
    
    // SHOW GRASP POSE (result from generate grasp functions)
    display_pose = display_pose * grasp_data_.grasp_pose_to_eef_pose_.inverse();
    visual_tools_->publishAxis(display_pose, 0.05, 0.005);
    visual_tools_->publishSphere(display_pose.translation(), rviz_visual_tools::LIME_GREEN, 0.02);
    visual_tools_->publishText(display_pose, "Grasp Pose", rviz_visual_tools::WHITE, rviz_visual_tools::SMALL, text);

    // SHOW finger_to_palm_depth
    Eigen::Vector3d grasp_point = display_pose.translation();
    Eigen::Vector3d obj_point = visual_tools_->convertPose(cuboid_pose_).translation();
    Eigen::Vector3d palm_vector = obj_point - grasp_point;
    palm_vector.normalize();
    Eigen::Vector3d palm_point = grasp_point + palm_vector * grasp_data_.finger_to_palm_depth_;
    visual_tools_->publishLine(grasp_point, palm_point, rviz_visual_tools::GREY);

    Eigen::Vector3d text_translation = display_pose * palm_vector * (0.5 * grasp_data_.finger_to_palm_depth_);
    display_pose.translation() += text_translation;
    visual_tools_->publishText(display_pose, "finger_to_palm_depth", rviz_visual_tools::WHITE, rviz_visual_tools::SMALL, text);    
    
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

  bool verbose = false;

  moveit_grasps::GraspPosesVisualizer visualizer(verbose);

  return 0;
}
