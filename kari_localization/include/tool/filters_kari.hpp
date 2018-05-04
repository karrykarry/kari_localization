#ifndef _FILTERS_KARI_HPP_
#define _FILTERS_KARI_HPP_

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>



void map_reader(std::string file,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_map_cloud);

// void voxel_grid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);
void voxel_grid(float size,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);



#endif
