#ifndef POINTCLOUDTOMESH_H
#define POINTCLOUDTOMESH_H

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

//template <typename PointT, typename PointTNormal>
class PointCloudToMesh
{
	public:
	
	PointCloudToMesh() { }

	
	void set_input(const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud_input)
	{
		point_cloud = point_cloud_input;
	}
	
	bool compute_mesh()
	{
		if (!point_cloud.get()) return false;
		
		if (point_cloud->size() <= 0) return false;
		
		std::cout << "Dimensione Point Cloud: "<< point_cloud->size() << std::endl;
		
		//RIMUOVO VALORI NAN
				
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_nan  (new pcl::PointCloud<pcl::PointXYZ>);
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*point_cloud, *cloud_no_nan, indices);
		
		std::cout << "Dimensione Point Cloud senza valori NaN: " << cloud_no_nan->size() << std::endl;
		
		/*
		//RIMUOVO OUTLIER
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_outlier (new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		outrem.setInputCloud(cloud_no_nan);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
 		outrem.setKeepOrganized(true);
    // apply filter
    outrem.filter (*cloud_no_outlier);
		*/
		//APPLICO VOXELGRID PER FARE UN DOWNSAMPLING
		
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxelized (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::VoxelGrid<pcl::PointXYZ> gridFilter;
		gridFilter.setInputCloud (cloud_no_nan);
		gridFilter.setLeafSize (0.05f, 0.05f, 0.05f);
		gridFilter.filter (*cloud_voxelized);
		std::cout << "Numero di punti dopo aver applicato il filtro " << cloud_voxelized->size() << std::endl;
		
		//STIMO LA NORMALE PER OGNI PUNTO
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_ne (new pcl::search::KdTree<pcl::PointXYZ>);
		tree_ne->setInputCloud (cloud_voxelized);
		
		pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud (cloud_voxelized);
		ne.setSearchMethod (tree_ne);
		ne.setKSearch (20);
		ne.compute (*normals);
		std::cout << "Normali stimate" << std::endl;
		
		//UNISCO LA POINT CLOUD CON LE NORMALI CALCOLATE
	  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  	pcl::concatenateFields(*cloud_voxelized, *normals, *cloud_with_normals);
  	
  	pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
		tree->setInputCloud (cloud_with_normals);
		/*
		//COSTRUISCO LA MESH (MARCHING CUBE) -- NON E' MOLTO BELLO
		pcl::MarchingCubesHoppe<pcl::PointNormal> mc;
		mc.setIsoLevel (0);
		mc.setGridResolution (30,30,30);
		mc.setPercentageExtendGrid (0.3f);
		//pcl::MarchingCubesRBF<pcl::PointNormal> mc;
		//pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh);
		mc.setInputCloud (cloud_with_normals);
		mc.setSearchMethod (tree);
		mc.reconstruct (mesh);
		*/
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
	
	pcl::PolygonMesh getMesh()
	{
		return mesh;
	}	
	private:
	
	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud;
	
	pcl::PolygonMesh mesh;


};

#endif
