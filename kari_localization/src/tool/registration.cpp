#include "registration.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

Eigen::Matrix4f registration_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src){
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);

	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	return icp.getFinalTransformation();
}

Eigen::Matrix4f registration_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src){
	pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setMaximumIterations(100);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);

	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);
	pcl::PointCloud<pcl::PointXYZI> Final;
	icp.align(Final);

	return icp.getFinalTransformation();
}

Eigen::Matrix4f registration_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src){
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(35);

	/*------ Voxel Grid ------*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_tgt (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(0.5,0.5,0.5);
	vg.setInputCloud(cloud_src);
	vg.filter(*filtered_cloud_src);
	vg.setInputCloud(cloud_tgt);
	vg.filter(*filtered_cloud_tgt);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	ndt.setInputTarget(filtered_cloud_tgt);
	ndt.setInputSource(filtered_cloud_src);
	ndt.align(*cloud);

	return ndt.getFinalTransformation();
}



Eigen::Matrix4f registration_ndt_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	ndt.setResolution(1.0);
	ndt.setMaximumIterations(35);

	/*------ Voxel Grid ------*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_tgt (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(0.5,0.5,0.5);
	vg.setInputCloud(cloud_src);
	vg.filter(*filtered_cloud_src);
	vg.setInputCloud(cloud_tgt);
	vg.filter(*filtered_cloud_tgt);

	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	ndt.setInputTarget(filtered_cloud_tgt);
	ndt.setInputSource(filtered_cloud_src);
	ndt.align(*cloud);

	return ndt.getFinalTransformation();
}

