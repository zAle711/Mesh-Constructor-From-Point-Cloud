/*
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tf2_msgs/TFMessage.h"
#include "sensor_msgs/PointCloud2.h"

#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types.h"
#include "pcl/io/pcd_io.h"

pcl::PointCloud<pcl::PointXYZ> cloud;


void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
	pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
	pcl::fromROSMsg(*msg, pcl_cloud);
	cloud += pcl_cloud;
	//pcl::io::savePCDFileASCII("/home/output_point_cloud.pcd", cloud);
	
	ROS_INFO("Ricevuto messaggio di tipo PointCloud");
}

void tfCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
	for (const auto& transform : msg->transforms)
	{
		ROS_INFO("Frame ID: %s, Child Frame ID: %s", transform.header.frame_id.c_str(), transform.child_frame_id.c_str());
    ROS_INFO("Translation (x, y, z): %.2f, %.2f, %.2f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
    ROS_INFO("Rotation (x, y, z, w): %.2f, %.2f, %.2f, %.2f", transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
	}
}


void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
 

  ros::init(argc, argv, "listener");

 
  ros::NodeHandle n;

  //ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);
	//ros::Subscriber sub = n.subscribe("tf", 1000,  tfCallback);
	ros::Subscriber sub2 = n.subscribe("camera/rgb/points", 1000, pointCloudCallback);
  

  ros::spin();

  return 0;
} */


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
