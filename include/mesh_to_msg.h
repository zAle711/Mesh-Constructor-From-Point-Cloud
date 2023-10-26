#ifndef MESHTOMSG_H
#define MESHTOMSG_H

#include <shape_msgs/Mesh.h>
#include <pcl_msgs/PolygonMesh.h>

#include <pcl/PolygonMesh.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_conversions/pcl_conversions.h>


static bool meshToShapeMsg(const pcl::PolygonMesh& input_mesh, shape_msgs::Mesh& shape_msg)
{
	
 	pcl_msgs::PolygonMesh pcl_msg_mesh;
  pcl_conversions::fromPCL(input_mesh, pcl_msg_mesh);


  sensor_msgs::PointCloud2Modifier pcd_modifier(pcl_msg_mesh.cloud);

  size_t size = pcd_modifier.size();

  shape_msg.vertices.resize(size);

  std::cout << "polys: " << pcl_msg_mesh.polygons.size() << " vertices: " << pcd_modifier.size() << "\n";

  sensor_msgs::PointCloud2ConstIterator<float> pt_iter(pcl_msg_mesh.cloud, "x");

  for(size_t i = 0; i < size ; i++, ++pt_iter){
    shape_msg.vertices[i].x = pt_iter[0];
    shape_msg.vertices[i].y = pt_iter[1];
    shape_msg.vertices[i].z = pt_iter[2];
  }

  //ROS_INFO("Found %ld polygons", triangles.size());

  std::cout << "Updated vertices" << "\n";

  //BOOST_FOREACH(const Vertices polygon, triangles)

  shape_msg.triangles.resize(input_mesh.polygons.size());

  for (size_t i = 0; i < input_mesh.polygons.size(); ++i)
  {
    if(input_mesh.polygons[i].vertices.size() < 3)
    {
      ROS_WARN("Not enough points in polygon. Ignoring it.");
      continue;
    }

    //shape_msgs::MeshTriangle triangle = shape_msgs::MeshTriangle();
    //boost::array<uint32_t, 3> xyz = {{in.polygons[i].vertices[0], in.polygons[i].vertices[1], in.polygons[i].vertices[2]}};
    //triangle.vertex_indices = xyz;

    //mesh.triangles.push_back(shape_msgs::MeshTriangle());
    //mesh.triangles[i].vertex_indices.resize(3);

    for (int j = 0; j < 3; ++j)
      shape_msg.triangles[i].vertex_indices[j] = input_mesh.polygons[i].vertices[j];
  }
  return true;

}


#endif
