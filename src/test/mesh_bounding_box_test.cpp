/*
  Author: Andy McEvoy
  Desc:   Tests the grasps.cpp bounding box function
*/

#include <iostream>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/grasp_generator.h>

#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

namespace fs = boost::filesystem;

namespace moveit_grasps
{
class MeshBoundingBoxTest
{
public:
  MeshBoundingBoxTest(int number_of_trials, bool verbose)
  {
    // setup rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base", "/rviz_visual_tools"));
    visual_tools_->loadMarkerPub();
    visual_tools_->deleteAllMarkers();

    // seed random
    srand(ros::Time::now().toSec());

    // load grasp generator
    grasp_generator_.reset(new GraspGenerator(visual_tools_, verbose));

    // int completed_trials = 0;

    ROS_INFO_STREAM_NAMED("test", "\n*************\nStarting test\n*************");

    std::string package_path = ros::package::getPath("picknik_main");
    fs::path target_dir(package_path + "/meshes/products/");

    fs::directory_iterator it(target_dir), eod;
    ROS_DEBUG_STREAM_NAMED("bbox", "Directory: " << target_dir.string());

    double depth, width, height;
    Eigen::Affine3d pose_oobb;
    Eigen::Affine3d mesh_pose = Eigen::Affine3d::Identity();

    // loop through each product mesh and test bounding box
    BOOST_FOREACH (fs::path const& p, std::make_pair(it, eod))
    {
      if (!ros::ok())
        break;

      pose_oobb = Eigen::Affine3d::Identity();
      depth = 0;
      width = 0;
      height = 0;

      fs::path file_name("recommended.stl");
      fs::path mesh_path = p / file_name;
      ROS_INFO_STREAM_NAMED("bbox", "Processing mesh " << mesh_path.string());

      ROS_DEBUG_STREAM_NAMED("bbox", "creating mesh from file... ");
      shapes::Shape* mesh = shapes::createMeshFromResource("file://" + mesh_path.string());

      ROS_DEBUG_STREAM_NAMED("bbox", "creating shape message...");
      shapes::ShapeMsg shape_msg;
      if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
      {
        ROS_ERROR_STREAM_NAMED("bounding box test", "Unable to create mesh shape message from resource " << mesh_path);
      }
      shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

      ROS_DEBUG_STREAM_NAMED("bbox", "getting bounding box...");

      ROS_WARN_STREAM_NAMED("temp", "TODO bounding box");
      // grasp_generator_->getBoundingBoxFromMesh(mesh_msg, pose_oobb, depth, width, height); // this introduces a huge
      // delay
      visual_tools_->publishMesh(mesh_pose, "file://" + mesh_path.string());

      std::cin.get();
      visual_tools_->deleteAllMarkers();
    }
  }

private:
  GraspGeneratorPtr grasp_generator_;
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;

};  // class

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_bounding_box_test");

  int number_of_trials = 1;
  bool verbose = true;

  if (argc > 1)
  {
    for (std::size_t i = 0; i < std::size_t(argc); i++)
    {
      if (strcmp(argv[i], "--verbose") == 0)
      {
        i++;
        if (strcmp(argv[i], "true") == 0)
        {
          ROS_INFO_STREAM_NAMED("main", "Running in VERBOSE mode (slower)");
          verbose = true;
        }
        continue;
      }

      if (strcmp(argv[i], "--trials") == 0)
      {
        i++;
        number_of_trials = std::atoi(argv[i]);
        continue;
      }
    }
    ROS_INFO_STREAM_NAMED("main", "Will run " << number_of_trials << " trials");
  }

  moveit_grasps::MeshBoundingBoxTest tester(number_of_trials, verbose);

  return 0;
}
