#include "filters_kari.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>


void map_reader(std::string file,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_map_cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile(file, *target_map_cloud);

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (0.5, 0.5, 0.5);
    approximate_voxel_filter.setInputCloud (target_map_cloud);
    approximate_voxel_filter.filter (*filtered_map_cloud);

}


void voxel_grid(float size,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud){
// void voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud){

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (size, size, size);
	// approximate_voxel_filter.setLeafSize (0.5, 0.5, 0.5);
    approximate_voxel_filter.setInputCloud (cloud);
    approximate_voxel_filter.filter (*filtered_cloud);

}
