/*
  Author: Andy McEvoy
  Desc:   Tests the grasps.cpp bounding box function
*/

#include <ros/ros.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/grasps.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/bodies.h>

//#include <pcl17/features/moment_of_inertia_estimation.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace moveit_grasps
{

class MeshBoundingBoxTest
{
private:
  moveit_visual_tools::MoveItVisualToolsPtr visual_tools_;
  geometry_msgs::Pose mesh_pose_;

public:

  MeshBoundingBoxTest(int number_of_trials, bool verbose)
  {

    // setup rviz
    visual_tools_.reset(new moveit_visual_tools::MoveItVisualTools("base","/rviz_visual_tools"));
    visual_tools_->setMuted(false);
    visual_tools_->loadMarkerPub();
    visual_tools_->deleteAllMarkers();

    // seed random
    srand(ros::Time::now().toSec());
    
    int completed_trials = 0;
    
    while(ros::ok())
    {
      ROS_INFO_STREAM_NAMED("test","\n************* \nStarting test " 
                            << completed_trials + 1 << " of " << number_of_trials << "\n*************");

      // get mesh file to publish
      // TODO: grab a random mesh from the meshes directory
      fs::path mesh_path = "file:/home/andy/ros/ws_picknik/src/picknik/picknik_main/meshes/products/kong_air_dog_squeakair_tennis_ball/recommended.dae";
      
      // get random pose for mesh
      rviz_visual_tools::RandomPoseBounds bounds(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
      visual_tools_->generateRandomPose(mesh_pose_,bounds );
      ROS_INFO_STREAM_NAMED("test","Position = " << mesh_pose_.position.x << ", " << 
                            mesh_pose_.position.y << ", " << mesh_pose_.position.z);
      ROS_INFO_STREAM_NAMED("test","Quaternion = " << mesh_pose_.orientation.x << ", " <<
                            mesh_pose_.orientation.y << ", " << mesh_pose_.orientation.z);

      visual_tools_->publishMesh(mesh_pose_,mesh_path.string());

      // GET BOUNDING BOX FUNCTION
<<<<<<< Updated upstream
      shapes::Shape *mesh = shapes::createMeshFromResource(mesh_path.string());
=======
      shapes::Shape* mesh = shapes::createMeshFromResource(mesh_path.string());

      const bodies::Body* body = new bodies::ConvexMesh(mesh);

      double depth, width, height;
      computeBoundingBox(body, depth, width, height);
>>>>>>> Stashed changes
       
      // END BOUNDING BOX FUNCTION

      completed_trials++;
      if (completed_trials == number_of_trials)
        break;
    }

  }

<<<<<<< Updated upstream
=======
  void computeBoundingBox(const bodies::Body* body, double& depth, double& width, double& height)
  {
    const bodies::ConvexMesh* conv = dynamic_cast<const bodies::ConvexMesh*>(body);

    size_t number_of_vertices = conv->getScaledVertices().size();
    ROS_DEBUG_STREAM_NAMED("bounding_box","Number of vertices = " << number_of_vertices);

    double xmin, xmax, ymin, ymax, zmin, zmax;

    for (int i = 0; i < number_of_vertices; i++)
    {

    }

  }

>>>>>>> Stashed changes
  double fRand(double fMin, double fMax) 
  {
    return fMin + ( (double)rand() / RAND_MAX ) * (fMax - fMin);
  }


}; // class

} // namespace
 
 int main(int argc, char** argv)
{
  ros::init(argc, argv, "mesh_bounding_box_test");

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
    ROS_INFO_STREAM_NAMED("main","Will run " << number_of_trials << " trials");
  }

  moveit_grasps::MeshBoundingBoxTest tester(number_of_trials, verbose);

  return 0;
}

