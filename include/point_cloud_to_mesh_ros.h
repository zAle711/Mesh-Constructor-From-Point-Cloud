#ifndef POINTCLOUDTOMESHROS_H
#define POINTCLOUDTOMESHROS_H

#include <point_cloud_to_mesh.h>
#include <mesh_to_msg.h>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <tf/transform_listener.h>
//PCL 
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>


//template <typename PointT>
class PointCloudToMeshRos
{
public:
	PointCloudToMeshRos() 
	{
		ros::NodeHandle nh;
		
		cloud_sub = nh.subscribe("camera/rgb/points", 1, &PointCloudToMeshRos::cloudCallBack, this);
		
		shape_pub = nh.advertise<shape_msgs::Mesh>("mesh_shape", 1, true);
		f_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_point_cloud",1, true);
		
		
		
	}
		
	void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
		ROS_INFO("Messaggio Arrivato!");
		
		sensor_msgs::PointCloud2 cloud_out;
		tf::StampedTransform transform;
		
		bool transform_done = pcl_ros::transformPointCloud("world", *cloud_in, cloud_out, listener);
		if (!transform_done) 
		{
			listener.waitForTransform("world", "openni_rgb_optical_frame", ros::Time(0), ros::Duration(0.5));
			if (pcl_ros::transformPointCloud("world", *cloud_in, cloud_out, listener)) 
			{
				ROS_INFO("Message Discarded. Can't transform from /openni_rgb_optical_frame --> /world ");
				return;
			}
		}
		
		/*
		while (true) {
			
			if (pcl_ros::transformPointCloud("/world", *cloud_in, cloud_out, listener)) break;
			std::cout << "Aspetto la trasformata per 0.5 secondi" << std::endl;
			listener.waitForTransform("/world", "/openni_rgb_optical_frame", ros::Time(0), ros::Duration(0.5));
		}
		
		while (!pcl_ros::transformPointCloud("/world", *cloud_in, cloud_out, listener)) {
			std::cout << "dentro la while per la trasformata!" << std::endl;
		}
		
				 
		//listener.lookupTransform(cloud->header.frame_id, "world", 
		//std::cout << cloud_in->header.frame_id << std::endl;
		*/
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(cloud_out, *point_cloud);
		
		cloud_to_mesh.set_input(point_cloud);
		
		if (cloud_to_mesh.compute_mesh())
		{
			
			if (shape_pub.getNumSubscribers() > 0)
			{
				shape_msgs::Mesh mesh_msg;
				
				meshToShapeMsg(cloud_to_mesh.getMesh(), mesh_msg);
				shape_pub.publish(mesh_msg);
				ROS_INFO("Pubblicata Mesh Ricostruita");
			
			}
			
		

		
			if (f_cloud_pub.getNumSubscribers() > 0 )
			{
				sensor_msgs::PointCloud2 point_cloud_msg;
				pcl::toROSMsg(cloud_to_mesh.getGlobalPointCloud(), point_cloud_msg);
				f_cloud_pub.publish(point_cloud_msg);
				ROS_INFO("Pubblicata Point Cloud Filtrata");
			
			}
		
		}
	}
	

private:
	ros::Subscriber cloud_sub;
	ros::Publisher shape_pub;
	ros::Publisher f_cloud_pub;
	tf::TransformListener listener;
	
	PointCloudToMesh cloud_to_mesh;
	
};

#endif

