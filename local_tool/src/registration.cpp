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
//intensity
Eigen::Matrix4f registration_icp_I(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src){
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

Eigen::Matrix4f registration_icp_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud){
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(1.0);		//対応距離の最大値
	icp.setMaximumIterations(100); 				//ICPの最大繰り返し回数
	icp.setTransformationEpsilon(1e-8);			//RANSAC除去距離
	icp.setEuclideanFitnessEpsilon(1e-8);		//変換パラメータ値

	icp.setInputSource(cloud_src);
	icp.setInputTarget(cloud_tgt);
	icp.align(*cloud);

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




//outputをvisual化
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



//outputをvisual化
Eigen::Matrix4f map_icp_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::Odometry odo){
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setMaxCorrespondenceDistance(0.5);
	icp.setMaximumIterations(20);
	icp.setTransformationEpsilon(1e-8);
	icp.setEuclideanFitnessEpsilon(1e-8);

	/*------ Voxel Grid ------*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_src (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_tgt (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::VoxelGrid<pcl::PointXYZ> vg;
	vg.setLeafSize(0.3,0.3,0.3);
	vg.setInputCloud(cloud_src);
	vg.filter(*filtered_cloud_src);
	vg.setInputCloud(cloud_tgt);
	vg.filter(*filtered_cloud_tgt);


	Eigen::AngleAxisf init_rotation (odo.pose.pose.orientation.z , Eigen::Vector3f::UnitZ ());
	Eigen::Translation3f init_translation (odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z);

	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();



	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	icp.setInputTarget(filtered_cloud_tgt);
	icp.setInputSource(filtered_cloud_src);
	// ndt.align(*cloud);
    icp.align (*cloud, init_guess);

	return icp.getFinalTransformation();
}


Eigen::Matrix4f map_ndt_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, nav_msgs::Odometry odo){
	pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
	ndt.setTransformationEpsilon(0.001);
	ndt.setStepSize(0.1);
	ndt.setResolution(2.0);//1.0 change 05/09
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


	Eigen::AngleAxisf init_rotation (odo.pose.pose.orientation.z , Eigen::Vector3f::UnitZ ());
	Eigen::Translation3f init_translation (odo.pose.pose.position.x, odo.pose.pose.position.y, odo.pose.pose.position.z);

	Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();



	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	ndt.setInputTarget(filtered_cloud_tgt);
	ndt.setInputSource(filtered_cloud_src);
	// ndt.align(*cloud);
    ndt.align (*cloud, init_guess);

	return ndt.getFinalTransformation();
}
