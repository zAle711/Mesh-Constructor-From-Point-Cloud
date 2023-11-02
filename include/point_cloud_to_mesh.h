#ifndef POINTCLOUDTOMESH_H
#define POINTCLOUDTOMESH_H

#include <string> 

#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/search/kdtree.h> 
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/pcd_io.h>

//template <typename PointT, typename PointTNormal>
class PointCloudToMesh
{
	public:
	
	PointCloudToMesh() { }

	
	void set_input(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud_input)
	{
		new_point_cloud = point_cloud_input;
	}
	
	
	
	bool compute_mesh()
	{
		
		if (new_point_cloud->size() <= 0) return false;
		
		if (!FilterPointCloud()) {
			ROS_WARN("Point Cloud Downsampling Failed!");
			return false;
		}
		
		new_point_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
		
		//STIMO LA NORMALE PER OGNI PUNTO
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ne (new pcl::search::KdTree<pcl::PointXYZ>);
		tree_ne->setInputCloud (filtered_point_cloud);
		
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (filtered_point_cloud);
		ne.setSearchMethod (tree_ne);
		ne.setKSearch (20);
		ne.compute (*normals);
		//std::cout << "Normali stimate" << std::endl;
		
		//UNISCO LA POINT CLOUD CON LE NORMALI CALCOLATE
	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  	pcl::concatenateFields(*filtered_point_cloud, *normals, *cloud_with_normals);
  	
  	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
		tree->setInputCloud (cloud_with_normals);
		
		//COSTRUISCO LA MESH GREEDPROYECTION
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		
		gp3.setSearchRadius(0.1);
		gp3.setMu (2.5);
		gp3.setMaximumNearestNeighbors(50);
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(true);
		gp3.setConsistentVertexOrdering(true);
		
		gp3.setInputCloud (cloud_with_normals);
	  gp3.setSearchMethod (tree);
	  gp3.reconstruct (mesh);
		
		return true;
	}
	
	bool FilterPointCloud()
	{
		if (!new_point_cloud.get()) return false;
		
		if (new_point_cloud->size() <= 0) return false;
		
		//Rimuovo valori NaN
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nan  (new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*new_point_cloud, *cloud_no_nan, indices);
		
		//VoxelGrid DownSampling
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> gridFilter;
		gridFilter.setInputCloud (cloud_no_nan);
		gridFilter.setLeafSize (0.05f, 0.05f, 0.05f);
		gridFilter.filter (*cloud_voxelized);
		filtered_point_cloud = cloud_voxelized;
		
		if (point_cloud.size() == 0)
		{
			point_cloud = *filtered_point_cloud;
		} 
		else
		{
			
			point_cloud += *filtered_point_cloud;
		}
	
		ROS_INFO("Point Cloud original size: %lu, after removing NaN: %lu, after filtering: %lu", new_point_cloud->size(), cloud_no_nan->size(), cloud_voxelized->size());
		
		pcl::io::savePCDFileASCII ("/home/all_points.pcd", point_cloud);
		
		return true;
	}
	
	bool UpdatePointCloud()
	{
		return false;
	}
	
	
	pcl::PolygonMesh getMesh()
	{
		return mesh;
	}	
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr getFilteredPointCloud()
	{
		return 	filtered_point_cloud;
	}
	
	pcl::PointCloud<pcl::PointXYZ> getGlobalPointCloud()
	{
		return point_cloud;
	}
	private:
	
	pcl::PointCloud<pcl::PointXYZ> point_cloud;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> new_point_cloud;
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> filtered_point_cloud;
	
	pcl::PolygonMesh mesh;


};

#endif
