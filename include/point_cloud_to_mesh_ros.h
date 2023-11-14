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

class PointCloudToMeshRos
{
public:

	PointCloudToMeshRos()
	{
		ros::NodeHandle nh;

        nh.param("/point_cloud_recorder/waiting_time", waiting_time, 0.5);
        nh.param("/point_cloud_recorder/queue_size", queue_size, 1);
        nh.param("/point_cloud_recorder/save_to_file", save_to_file, false);
        nh.param("/point_cloud_recorder/chunkSize", chunkSize, 16);
        nh.param("/point_cloud_recorder/offset", offset, 10.0f);
        nh.param("/point_cloud_recorder/precision", precision, 100.0f);


		cloud_sub = nh.subscribe("cloud_in", queue_size, &PointCloudToMeshRos::updatePointCloud, this);

		//shape_pub = nh.advertise<shape_msgs::Mesh>("mesh_shape", 1, true);
		//marker_pub = nh.advertise<visualization_msgs::Marker>("mesh_marker", 1, true);

		cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("global_point_cloud",1, true);
		points_pub = nh.advertise<geometry_msgs::PoseArray>("points_array",1 , true);

		voxelWorld = new World(chunkSize, offset, precision);

	}

	void updatePointCloud(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
	{
	    total_point_cloud_received += 1;
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

        point_cloud_registered += 1;

		pcl::PointCloud<pcl::PointXYZ>::Ptr new_point_cloud (new pcl::PointCloud<pcl::PointXYZ>());
		pcl::fromROSMsg(cloud_out, *new_point_cloud);

        geometry_msgs::PoseArray  posearray;
        posearray.header.stamp = ros::Time::now(); // timestamp of creation of the msg
        posearray.header.frame_id = "world";
        int i = 0;
	    for (auto elem : new_point_cloud->points)
        {

            if (pcl::isFinite(elem))
            {
                i += 1;
                int x = (int) ( (elem.x + offset) * precision );
                int y = (int) ( (elem.y + offset) * precision );
                int z = (int) ( (elem.z + offset) * precision );
                //ROS_INFO("%f %f %f %d %d %d", elem.x, elem.y, elem.z, x, y, z);

                if (voxelWorld->CheckBlock(x, y, z))
                {

                    geometry_msgs::Pose p;
                    p.position.x = x;
                    p.position.y = y;
                    p.position.z = z;

                    p.orientation.x = 0;
                    p.orientation.y = 0;
                    p.orientation.z = 0;
                    p.orientation.w = 0;

                    posearray.poses.push_back(p);
                }
                /*
                int x = (int) (elem.x * precision);
                int y = (int) (elem.y * precision);
                int z = (int) (elem.z * precision);

                std::string key = std::to_string(x) + "-" + std::to_string(y) + "-" + std::to_string(z);
                //ROS_INFO("%s", key.c_str());
                /*
                int set_size = uniquePoints.size();
                uniquePoints.insert(key);
                if (set_size != uniquePoints.size())
                {
                    geometry_msgs::Pose p;
                    p.position.x = x / precision;
                    p.position.y = y / precision;
                    p.position.z = z / precision;

                    p.orientation.x = 0;
                    p.orientation.y = 0;
                    p.orientation.z = 0;
                    p.orientation.w = 0;


                    posearray.poses.push_back(p);
                    //if (uniquePoints.size() % 345 == 0) ROS_INFO("%f, %f, %f", p.position.x, p.position.y, p.position.z);
                }
                */
             }

        }
        ROS_INFO("Point Cloud Traslated %d/%d, Pose array size: %lu/%d", point_cloud_registered, total_point_cloud_received, posearray.poses.size(), i);
        points_pub.publish(posearray);
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

				if (save_to_file) cloud_to_mesh.save_to_file(point_cloud_registered);

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
    float offset;
    float precision;

	int point_cloud_registered = 0;
	int total_point_cloud_received = 0;

	double waiting_time;
	int queue_size;
	bool save_to_file;

};

#endif

