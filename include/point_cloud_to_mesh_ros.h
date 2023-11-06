#ifndef POINTCLOUDTOMESHROS_H
#define POINTCLOUDTOMESHROS_H

#include <point_cloud_to_mesh.h>
#include <mesh_to_msg.h>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <visualization_msgs/Marker.h>
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

		cloud_sub = nh.subscribe("camera/rgb/points", 100, &PointCloudToMeshRos::registerPointCloudCallBack, this);

		shape_pub = nh.advertise<shape_msgs::Mesh>("mesh_shape", 1, true);
		marker_pub = nh.advertise<visualization_msgs::Marker>("mesh_marker", 1, true);
		cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_point_cloud",1, true);



	}

	void registerPointCloudCallBack (const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
        ROS_INFO("New Point Cloud Received");

        //Transform Point Cloud with TF Info
        sensor_msgs::PointCloud2 cloud_out;
		tf::StampedTransform transform;

		bool transform_done = pcl_ros::transformPointCloud("world", *cloud_in, cloud_out, listener);
		if (!transform_done)
		{
			listener.waitForTransform("world", cloud_in->header.frame_id, ros::Time(0), ros::Duration(0.5)); //ros::Time::now()
			ROS_INFO("Waiting for Transform (0.5s)");
			if (!pcl_ros::transformPointCloud("world", *cloud_in, cloud_out, listener))
			{
				ROS_INFO("Message Discarded. Can't transform from %s --> /world ", cloud_in->header.frame_id.c_str());
				return;
			}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr new_point_cloud (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(cloud_out, *new_point_cloud);

		if (cloud_to_mesh.register_point_cloud(new_point_cloud))
        {
            if (cloud_pub.getNumSubscribers() > 0 )
            {
                sensor_msgs::PointCloud2 point_cloud_msg;
				pcl::toROSMsg(cloud_to_mesh.getGlobalPointCloud(), point_cloud_msg);
				cloud_pub.publish(point_cloud_msg);
				ROS_INFO("Global Point Cloud published");
            }
        }
        else
        {
            ROS_ERROR ("Error while registering the point cloud");
            return;
        }

        if (cloud_to_mesh.compute_mesh())
        {
            if (shape_pub.getNumSubscribers() > 0)
            {
                shape_msgs::Mesh mesh_msg;
				meshToShapeMsg(cloud_to_mesh.getMesh(), mesh_msg);
				shape_pub.publish(mesh_msg);
				ROS_INFO("Shape_msg::Mesh Published!");
            }

            if (marker_pub.getNumSubscribers() > 0)
            {
                visualization_msgs::Marker marker_msg;
                meshToMarkerMsg(cloud_to_mesh.getMesh(), marker_msg);
                marker_pub.publish(marker_msg);
                ROS_INFO("Visualization_msg::Marker Published!");

            }

        }
	}
    /*
	void cloudCallBack(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
		ROS_INFO("Messaggio Arrivato!");

        //std::cout << cloud_in->header.frame_id << std::endl;

		sensor_msgs::PointCloud2 cloud_out;
		tf::StampedTransform transform;

		bool transform_done = pcl_ros::transformPointCloud("world", *cloud_in, cloud_out, listener);
		if (!transform_done)
		{
			listener.waitForTransform("world", "openni_rgb_optical_frame", ros::Time(0), ros::Duration(0.5));
			ROS_INFO("Waiting for Transform (0.5s)");
			//listener.waitForTransform("world", "openni_rgb_optical_frame", ros::Time::now(), ros::Duration(0.5));
			if (!pcl_ros::transformPointCloud("world", *cloud_in, cloud_out, listener))
			{
				ROS_INFO("Message Discarded. Can't transform from /openni_rgb_optical_frame --> /world ");
				return;
			}
		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud (new pcl::PointCloud<pcl::PointXYZ>());
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




			if (cloud_pub.getNumSubscribers() > 0 )
			{
				sensor_msgs::PointCloud2 point_cloud_msg;
				pcl::toROSMsg(cloud_to_mesh.getGlobalPointCloud(), point_cloud_msg);
				cloud_pub.publish(point_cloud_msg);
				ROS_INFO("Pubblicata Point Cloud Filtrata");

			}

		}
	}
    */

private:
	ros::Subscriber cloud_sub;

	ros::Publisher cloud_pub;
	ros::Publisher shape_pub;
	ros::Publisher marker_pub;

	tf::TransformListener listener;

	PointCloudToMesh cloud_to_mesh;

};

#endif

