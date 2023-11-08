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

    bool register_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_in)
    {
        if (!point_cloud_in.get()) return false;

        if (point_cloud_in->size() <= 0) return false;

        //Rimuovo valori NaN
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nan  (new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*point_cloud_in, *cloud_no_nan, indices);

        //Filter Point Cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> gridFilter;
		gridFilter.setInputCloud (cloud_no_nan);
		gridFilter.setLeafSize (0.05f, 0.05f, 0.05f);
		gridFilter.filter (*cloud_voxelized);

        if (global_point_cloud.size() == 0)
            global_point_cloud = *cloud_voxelized;
        else
            global_point_cloud += *cloud_voxelized;

        return true;
    }

	bool compute_mesh()
	{

		if (global_point_cloud.size() <= 0) return false;

        pcl::PointCloud<pcl::PointXYZ>::Ptr gpc = global_point_cloud.makeShared();

		//STIMO LA NORMALE PER OGNI PUNTO
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ne (new pcl::search::KdTree<pcl::PointXYZ>);
		tree_ne->setInputCloud (gpc);

		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (gpc);
		ne.setSearchMethod (tree_ne);
		ne.setKSearch (20);
		ne.compute (*normals);

		//UNISCO LA POINT CLOUD CON LE NORMALI CALCOLATE
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
        pcl::concatenateFields(*gpc, *normals, *cloud_with_normals);

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

	void save_to_file()
	{
        pcl::io::savePCDFileASCII ("/home/global_point_cloud.pcd", global_point_cloud);
	}

    void save_to_file(int n_cloud)
    {
        pcl::io::savePCDFileASCII ("/home/clouds_from_node/single_clouds/" + std::to_string(n_cloud) + ".pcd", global_point_cloud);
    }

	pcl::PolygonMesh getMesh()
	{
		return mesh;
	}

	pcl::PointCloud<pcl::PointXYZ> getGlobalPointCloud()
	{
		return global_point_cloud;
	}

	private:

	pcl::PointCloud<pcl::PointXYZ> global_point_cloud;

	pcl::PolygonMesh mesh;

};

#endif
