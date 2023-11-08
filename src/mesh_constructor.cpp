#include <ros/ros.h>
#include <pcl/point_types.h>
#include <point_cloud_to_mesh_ros.h>
#include <point_cloud_to_mesh.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_to_mesh_node");

  PointCloudToMeshRos conv;

  ros::spin();
  return 0;
}
