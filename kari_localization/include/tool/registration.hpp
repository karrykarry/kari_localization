#ifndef _REGISTRATION_HPP_
#define _REGISTRATION_HPP_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

Eigen::Matrix4f registration_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src);
Eigen::Matrix4f registration_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src);
Eigen::Matrix4f registration_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src);
Eigen::Matrix4f registration_ndt_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

#endif

