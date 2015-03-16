/*
  Author: Andy McEvoy
  Desc:   Tests the grasps.cpp bounding box function
*/

#include <ros/ros.h>


#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_grasps/grasps.h>

// #include <rviz_visual_tools/rviz_visual_tools.h>

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/bodies.h>

#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common_headers.h>

#include <boost/filesystem.hpp>

#include <limits>

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
      fs::path mesh_path = "file:/home/jorge/ws_picknik/src/picknik/picknik_main/meshes/products/kong_air_dog_squeakair_tennis_ball/recommended.dae";

      // get random pose for mesh
      rviz_visual_tools::RandomPoseBounds bounds(0.0, 1.0, 0.0, 1.0, 0.0, 1.0);
      visual_tools_->generateRandomPose(mesh_pose_,bounds );
      ROS_INFO_STREAM_NAMED("test","Position = " << mesh_pose_.position.x << ", " <<
                            mesh_pose_.position.y << ", " << mesh_pose_.position.z);
      ROS_INFO_STREAM_NAMED("test","Quaternion = " << mesh_pose_.orientation.x << ", " <<
                            mesh_pose_.orientation.y << ", " << mesh_pose_.orientation.z);

      visual_tools_->publishMesh(mesh_pose_,mesh_path.string());

      visual_tools_->publishAxis(mesh_pose_);
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

    double minx = std::numeric_limits<double>::max(), miny = std::numeric_limits<double>::max(), minz = std::numeric_limits<double>::max();
    double maxx = std::numeric_limits<double>::min(), maxy = std::numeric_limits<double>::min(), maxz = std::numeric_limits<double>::min();

    // get vertices into point cloud for pcl magic
    for (int i = 0; i < num_vertices; i++)
    {
      point.x = mesh_msg.vertices[i].x;
      point.z = mesh_msg.vertices[i].y;
      point.y = mesh_msg.vertices[i].z;
      // ROS_INFO_STREAM_NAMED("vertices", mesh_msg.vertices[i].x);
      // ROS_INFO_STREAM_NAMED("vertices", mesh_msg.vertices[i].y);
      // ROS_INFO_STREAM_NAMED("vertices", mesh_msg.vertices[i].z);
      cloud->points.push_back(point);
      if (point.x < minx) minx = point.x;
      if (point.x > maxx) maxx = point.x;

      if (point.y < miny) miny = point.y;
      if (point.y > maxy) maxy = point.y;

      if (point.z < minz) minz = point.z;
      if (point.z > maxz) maxz = point.z;

    }
    ROS_INFO_STREAM(minx);
    ROS_INFO_STREAM(miny);
    ROS_INFO_STREAM(minz);

    Eigen::Vector3d min_sphere (minx, miny, minz);
    Eigen::Vector3d max_sphere (maxx, maxy, maxz);

    visual_tools_->publishSphere(min_sphere, rviz_visual_tools::BLUE, rviz_visual_tools::LARGE);
    visual_tools_->publishSphere(max_sphere, rviz_visual_tools::RED, rviz_visual_tools::LARGE);

    ROS_INFO_STREAM(maxx);
    ROS_INFO_STREAM(maxy);
    ROS_INFO_STREAM(maxz);

    ROS_DEBUG_STREAM_NAMED("bbox","num points in cloud = " << cloud->points.size());

    // extract bounding box properties
    ROS_DEBUG_STREAM_NAMED("bbox","starting cloud feature extractor");

    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;
    feature_extractor.setInputCloud(cloud);
    feature_extractor.compute();

    // Oriented BoundingBox
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;

    feature_extractor.getOBB (min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);

    Eigen::Vector3f position (position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat (rotational_matrix_OBB);

     publishWireframeCuboid(visual_tools_->convertPose(mesh_pose_),
     			   position, rotational_matrix_OBB, min_point_OBB, max_point_OBB);

    // Axis oriented bounding box
    // pcl::PointXYZ min_point_AABB;
    // pcl::PointXYZ max_point_AABB;
    // feature_extractor.getAABB(min_point_AABB, max_point_AABB);

    // Eigen::Vector3f AABB_position = Eigen::Vector3f::Zero(3);
    // Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity(3,3);

    // ROS_INFO_STREAM(mesh_pose_);
    // ROS_INFO_STREAM(AABB_position);
    // ROS_INFO_STREAM(rotation_matrix);
    // ROS_INFO_STREAM(min_point_AABB);
    // ROS_INFO_STREAM(max_point_AABB);

    // publishWireframeCuboid(visual_tools_->convertPose(mesh_pose_),
    // 			   AABB_position, rotation_matrix, min_point_AABB, max_point_AABB);


    ROS_DEBUG_STREAM_NAMED("bbox","done extracting features");

  }

  void publishWireframeCuboid(const Eigen::Affine3d &pose,
			      const Eigen::Vector3f &position,
			      const Eigen::Matrix3f &rotation_matrix,
			      const pcl::PointXYZ &min_point,
			      const pcl::PointXYZ &max_point) {
    Eigen::Vector3f p1 (min_point.x, min_point.y, min_point.z);
    Eigen::Vector3f p2 (min_point.x, min_point.y, max_point.z);
    Eigen::Vector3f p3 (max_point.x, min_point.y, max_point.z);
    Eigen::Vector3f p4 (max_point.x, min_point.y, min_point.z);
    Eigen::Vector3f p5 (min_point.x, max_point.y, min_point.z);
    Eigen::Vector3f p6 (min_point.x, max_point.y, max_point.z);
    Eigen::Vector3f p7 (max_point.x, max_point.y, max_point.z);
    Eigen::Vector3f p8 (max_point.x, max_point.y, min_point.z);

    p1 = rotation_matrix * p1 + position;
    p2 = rotation_matrix * p2 + position;
    p3 = rotation_matrix * p3 + position;
    p4 = rotation_matrix * p4 + position;
    p5 = rotation_matrix * p5 + position;
    p6 = rotation_matrix * p6 + position;
    p7 = rotation_matrix * p7 + position;
    p8 = rotation_matrix * p8 + position;

    Eigen::Vector3d pt1 = p1.cast <double> ();
    Eigen::Vector3d pt2 = p2.cast <double> ();
    Eigen::Vector3d pt3 = p3.cast <double> ();
    Eigen::Vector3d pt4 = p4.cast <double> ();
    Eigen::Vector3d pt5 = p5.cast <double> ();
    Eigen::Vector3d pt6 = p6.cast <double> ();
    Eigen::Vector3d pt7 = p7.cast <double> ();
    Eigen::Vector3d pt8 = p8.cast <double> ();



    visual_tools_->publishLine(pt1, pt2);
    visual_tools_->publishLine(pt1, pt4);
    visual_tools_->publishLine(pt1, pt5);
    visual_tools_->publishLine(pt5, pt6);
    visual_tools_->publishLine(pt5, pt8);
    visual_tools_->publishLine(pt2, pt6);
    visual_tools_->publishLine(pt6, pt7);
    visual_tools_->publishLine(pt7, pt8);
    visual_tools_->publishLine(pt2, pt3);
    visual_tools_->publishLine(pt4, pt8);
    visual_tools_->publishLine(pt3, pt4);
    visual_tools_->publishLine(pt3, pt7);
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
