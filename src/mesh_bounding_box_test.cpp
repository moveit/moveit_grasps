/*
  Author: Andy McEvoy
  Desc:   Tests the grasps.cpp bounding box function
*/

#include <ros/ros.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/grasps.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/bodies.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common_headers.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;

namespace moveit_grasps
{

bool publishWireframeCuboid(const Eigen::Vector3f &position,
                            const Eigen::Quaternionf &quat,
                            const Eigen::Vector3f &min_point,
                            const Eigen::Vector3f &max_point) {
  Eigen::Vector3f p1 (min_point.x, min_point.y, min_point.z);
  Eigen::Vector3f p2 (min_point.x, min_point.y, max_point.z);
  Eigen::Vector3f p3 (max_point.x, min_point.y, max_point.z);
  Eigen::Vector3f p4 (max_point.x, min_point.y, min_point.z);
  Eigen::Vector3f p5 (min_point.x, max_point.y, min_point.z);
  Eigen::Vector3f p6 (min_point.x, max_point.y, max_point.z);
  Eigen::Vector3f p7 (max_point.x, max_point.y, max_point.z);
  Eigen::Vector3f p8 (max_point.x, max_point.y, min_point.z);

  p1 = rotational_matrix_OBB * p1 + position;
  p2 = rotational_matrix_OBB * p2 + position;
  p3 = rotational_matrix_OBB * p3 + position;
  p4 = rotational_matrix_OBB * p4 + position;
  p5 = rotational_matrix_OBB * p5 + position;
  p6 = rotational_matrix_OBB * p6 + position;
  p7 = rotational_matrix_OBB * p7 + position;
  p8 = rotational_matrix_OBB * p8 + position;

  pcl::PointXYZ pt1 (p1 (0), p1 (1), p1 (2));
  pcl::PointXYZ pt2 (p2 (0), p2 (1), p2 (2));
  pcl::PointXYZ pt3 (p3 (0), p3 (1), p3 (2));
  pcl::PointXYZ pt4 (p4 (0), p4 (1), p4 (2));
  pcl::PointXYZ pt5 (p5 (0), p5 (1), p5 (2));
  pcl::PointXYZ pt6 (p6 (0), p6 (1), p6 (2));
  pcl::PointXYZ pt7 (p7 (0), p7 (1), p7 (2));
  pcl::PointXYZ pt8 (p8 (0), p8 (1), p8 (2));

  publishLine(pt1, pt2, 1.0, 0.0, 0.0, "1 edge");
  publishLine(pt1, pt4, 1.0, 0.0, 0.0, "2 edge");
  publishLine(pt1, pt5, 1.0, 0.0, 0.0, "3 edge");
  publishLine(pt5, pt6, 1.0, 0.0, 0.0, "4 edge");
  publishLine(pt5, pt8, 1.0, 0.0, 0.0, "5 edge");
  publishLine(pt2, pt6, 1.0, 0.0, 0.0, "6 edge");
  publishLine(pt6, pt7, 1.0, 0.0, 0.0, "7 edge");
  publishLine(pt7, pt8, 1.0, 0.0, 0.0, "8 edge");
  publishLine(pt2, pt3, 1.0, 0.0, 0.0, "9 edge");
  publishLine(pt4, pt8, 1.0, 0.0, 0.0, "10 edge");
  publishLine(pt3, pt4, 1.0, 0.0, 0.0, "11 edge");
  publishLine(pt3, pt7, 1.0, 0.0, 0.0, "12 edge");

  return true;
}


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
      shapes::Shape* mesh= shapes::createMeshFromResource(mesh_path.string());
      shapes::ShapeMsg shape_msg;
      if (!mesh || !shapes::constructMsgFromShape(mesh, shape_msg))
      {
        ROS_ERROR_STREAM_NAMED("bounding box test","Unable to create mesh shape message from resource " << mesh_path);
      }
      shape_msgs::Mesh mesh_msg = boost::get<shape_msgs::Mesh>(shape_msg);

      double depth, width, height;
      computeBoundingBox(mesh_msg, depth, width, height);
       
      // END BOUNDING BOX FUNCTION

      completed_trials++;
      if (completed_trials == number_of_trials)
        break;
    }

  }

  void computeBoundingBox(shape_msgs::Mesh mesh_msg, double& depth, double& width, double& height)
  {
    
    int num_vertices = mesh_msg.vertices.size();
    ROS_DEBUG_STREAM_NAMED("bbox","num triangles = " << mesh_msg.triangles.size());
    ROS_DEBUG_STREAM_NAMED("bbox","num vertices = " << num_vertices);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ point;

    // get vertices into point cloud for pcl magic
    for (int i = 0; i < num_vertices; i++)
    {
      point.x = mesh_msg.vertices[i].x;
      point.y = mesh_msg.vertices[i].y;
      point.z = mesh_msg.vertices[i].z;
      cloud->points.push_back(point);
    }

    ROS_DEBUG_STREAM_NAMED("bbox","num points in cloud = " << cloud->points.size());

    // extract bounding box properties
    ROS_DEBUG_STREAM_NAMED("bbox","starting cloud feature extractor");

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    // Oriented BoundingBox
    //    pcl::PointXYZ min_point_OBB;
    //    pcl::PointXYZ max_point_OBB;
    //    pcl::PointXYZ position_OBB;
    //    Eigen::Matrix3f rotational_matrix_OBB;

    //    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    //    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    //    Eigen::Quaternionf quat (rotational_matrix_OBB);

    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;

    feature_extractor.getAABB(min_point_AABB, max_point_AABB);

    ROS_DEBUG_STREAM_NAMED("bbox","done extracting features");

  }

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

