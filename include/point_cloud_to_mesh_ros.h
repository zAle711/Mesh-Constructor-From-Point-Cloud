#ifndef POINTCLOUDTOMESHROS_H
#define POINTCLOUDTOMESHROS_H

#include <point_cloud_to_mesh.h>
#include <mesh_to_msg.h>
//ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
//PCL
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

//#include <Chunk.h>
#include <cmath>

class PointCloudToMeshRos
{
public:

	PointCloudToMeshRos()
	{
		ros::NodeHandle nh;

        nh.param("/point_cloud_recorder/waiting_time", waiting_time, 0.5);
        nh.param("/point_cloud_recorder/queue_size", queue_size, 1);
        nh.param("/point_cloud_recorder/save_to_file", save_to_file, true);
        nh.param("/point_cloud_recorder/chunkSize", chunkSize, 16);

		cloud_sub = nh.subscribe("cloud_in", queue_size, &PointCloudToMeshRos::updatePointCloud, this);

		//shape_pub = nh.advertise<shape_msgs::Mesh>("mesh_shape", 1, true);
		//marker_pub = nh.advertise<visualization_msgs::Marker>("mesh_marker", 1, true);

		cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_point_cloud",1, true);
		points_pub = nh.advertise<geometry_msgs::PoseArray>("points_array",1 , true);

		voxelWorld = new World(chunkSize);

	}

	void updatePointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
	    total_point_cloud_received += 1;
	    //Transform Point Cloud with TF Info
        sensor_msgs::PointCloud2 cloud_out;
		tf::StampedTransform transform;

		bool transform_done = pcl_ros::transformPointCloud("map", *cloud_in, cloud_out, listener);
		if (!transform_done)
		{
			listener.waitForTransform("map", cloud_in->header.frame_id, ros::Time::now(), ros::Duration(waiting_time)); //ros::Time::now() ros::Time(0)
			ROS_INFO("Waiting for Transform (%.2fs)", waiting_time);
			if (!pcl_ros::transformPointCloud("map", *cloud_in, cloud_out, listener))
			{
				ROS_INFO("Message Discarded. Can't transform from %s --> /map ", cloud_in->header.frame_id.c_str());
				return;
			}
		}

        point_cloud_registered += 1;

		pcl::PointCloud<pcl::PointXYZ>::Ptr new_point_cloud (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(cloud_out, *new_point_cloud);

        geometry_msgs::PoseArray  posearray;
        posearray.header.stamp = ros::Time::now(); // timestamp of creation of the msg
        posearray.header.frame_id = "map";
        int i = 0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud (new pcl::PointCloud<pcl::PointXYZ>());
        cloud_to_mesh.filter_point_cloud(new_point_cloud, filtered_point_cloud);

        ROS_INFO("Original size: %lu, filtered size: %lu", new_point_cloud->size(),filtered_point_cloud->size());

	    for (auto elem : filtered_point_cloud->points)
        {
                //ROS_INFO("Punti ricevuti: %f %f %f", elem.x, elem.y, elem.z);
                i += 1;
                bool x_positive = elem.x >= 0;
                bool y_positive = elem.y >= 0;
                bool z_positive = elem.z >= 0;

                int chunk_pos_x = (int) elem.x;
                int chunk_pos_y = (int) elem.y;
                int chunk_pos_z = (int) elem.z;

                int pos_x = int(abs(elem.x)*100 - (floor(abs(elem.x)))*100);
                int pos_y = int(abs(elem.y)*100 - (floor(abs(elem.y)))*100);
                int pos_z = int(abs(elem.z)*100 - (floor(abs(elem.z)))*100);

                int chunkMultiplier = 100 / chunkSize;

                chunk_pos_x = x_positive ? chunk_pos_x * chunkMultiplier + pos_x / chunkSize : chunk_pos_x * chunkMultiplier - (pos_x / chunkSize) - 1;
                chunk_pos_y = y_positive ? chunk_pos_y * chunkMultiplier + pos_y / chunkSize : chunk_pos_y * chunkMultiplier - (pos_y / chunkSize) - 1;
                chunk_pos_z = z_positive ? chunk_pos_z * chunkMultiplier + pos_z / chunkSize : chunk_pos_z * chunkMultiplier - (pos_z / chunkSize) - 1;


                pos_x = pos_x < chunkSize ? pos_x : pos_x - (pos_x / chunkSize) * chunkSize;
                pos_y = pos_y < chunkSize ? pos_y : pos_y - (pos_y / chunkSize) * chunkSize;
                pos_z = pos_z < chunkSize ? pos_z : pos_z - (pos_z / chunkSize) * chunkSize;

                pos_x = x_positive ? pos_x : chunkSize - 1 - pos_x;
                pos_y = y_positive ? pos_y : chunkSize - 1 - pos_y;
                pos_z = z_positive ? pos_z : chunkSize - 1 - pos_z;

                //ROS_INFO("Punti Elaborati %d %d %d %d %d %d", chunk_pos_x, chunk_pos_y, chunk_pos_z, pos_x, pos_y, pos_z);

                if (voxelWorld->CheckBlock(chunk_pos_x, chunk_pos_y, chunk_pos_z, pos_x, pos_y, pos_z))
                {

                    geometry_msgs::Pose p;
                    p.position.x = x_positive ? (float) chunk_pos_x + (float) pos_x / (float) 100 : (float) chunk_pos_x - (float) pos_x / (float) 100;
                    p.position.y = y_positive ? (float) chunk_pos_y + (float) pos_y / (float) 100 : (float) chunk_pos_y - (float) pos_y / (float) 100;
                    p.position.z = z_positive ? (float) chunk_pos_z + (float) pos_z / (float) 100 : (float) chunk_pos_z - (float) pos_z / (float) 100;

                    //ROS_INFO("chunks %d_%d_%d indexs %d %d %d point: %f %f %f", chunk_pos_x, chunk_pos_y, chunk_pos_z, pos_x, pos_y, pos_z, p.position.x, p.position.y, p.position.z);

                    p.orientation.x = 0;
                    p.orientation.y = 0;
                    p.orientation.z = 0;
                    p.orientation.w = 0;

                    posearray.poses.push_back(p);
                }

        }
        ROS_INFO("Point Cloud Traslated %d/%d, Pose array size: %lu/%d", point_cloud_registered, total_point_cloud_received, posearray.poses.size(), i);

        if (posearray.poses.size() > 0)
        {
            points_pub.publish(posearray);
        }

	}

	void registerPointCloudCallBack (const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
	    total_point_cloud_received += 1;
        ROS_INFO("New Point Cloud Received %d", total_point_cloud_received);

        //Transform Point Cloud with TF Info
        sensor_msgs::PointCloud2 cloud_out;
		tf::StampedTransform transform;

		bool transform_done = pcl_ros::transformPointCloud("world", *cloud_in, cloud_out, listener);
		if (!transform_done)
		{
			listener.waitForTransform("world", cloud_in->header.frame_id, ros::Time::now(), ros::Duration(waiting_time)); //ros::Time::now() ros::Time(0)
			ROS_INFO("Waiting for Transform (%.2fs)", waiting_time);
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
            point_cloud_registered += 1;
            if (cloud_pub.getNumSubscribers() > 0 )
            {
                sensor_msgs::PointCloud2 point_cloud_msg;
				pcl::toROSMsg(cloud_to_mesh.getGlobalPointCloud(), point_cloud_msg);
				cloud_pub.publish(point_cloud_msg);

				if (true) cloud_to_mesh.save_to_file();

				ROS_INFO("Global Point Cloud published, merged point cloud: %d/%d", point_cloud_registered, total_point_cloud_received);
            }
        }
        else
        {
            ROS_ERROR ("Error while registering the point cloud");
            return;
        }

        if (false) //cloud_to_mesh.compute_mesh()
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

private:
	ros::Subscriber cloud_sub;

	ros::Publisher cloud_pub;
	ros::Publisher shape_pub;
	ros::Publisher marker_pub;
	ros::Publisher points_pub;

	tf::TransformListener listener;

	PointCloudToMesh cloud_to_mesh;
    //std::unordered_set<std::string> uniquePoints;

    World* voxelWorld;
    int chunkSize;

	int point_cloud_registered = 0;
	int total_point_cloud_received = 0;

	double waiting_time;
	int queue_size;
	bool save_to_file;

};

#endif

