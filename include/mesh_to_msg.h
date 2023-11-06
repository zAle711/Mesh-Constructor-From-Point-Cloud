#ifndef MESHTOMSG_H
#define MESHTOMSG_H

#include <shape_msgs/Mesh.h>
#include <visualization_msgs/Marker.h>

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

static bool meshToMarkerMsg(const pcl::PolygonMesh& in, visualization_msgs::Marker& marker)
{
  marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
  marker.header.frame_id = in.cloud.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.color.r = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.id = 1;
  marker.action = visualization_msgs::Marker::ADD;


  shape_msgs::Mesh shape_msg_mesh;

  meshToShapeMsg(in, shape_msg_mesh);

  size_t size_triangles = shape_msg_mesh.triangles.size();

  marker.points.resize(size_triangles*3);

  std::cout << "polys: " << size_triangles << " vertices: " << shape_msg_mesh.vertices.size() << "\n";

  size_t i = 0;

  for (size_t tri_index = 0; tri_index < size_triangles; ++tri_index){

    /*
    std::cout << shape_msg_mesh.triangles[tri_index].vertex_indices[0] <<  " " <<
                 shape_msg_mesh.triangles[tri_index].vertex_indices[1] <<  " " <<
                 shape_msg_mesh.triangles[tri_index].vertex_indices[2] << "\n";
    */

    marker.points[i]   = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[0]];
    marker.points[i+1] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[1]];
    marker.points[i+2] = shape_msg_mesh.vertices[shape_msg_mesh.triangles[tri_index].vertex_indices[2]];
    i = i + 3;

  }
    return true;
}
#endif
