#ifndef _REGISTRATION_HPP_
#define _REGISTRATION_HPP_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

//その場での比較
Eigen::Matrix4f registration_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src);
Eigen::Matrix4f registration_icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src);
Eigen::Matrix4f registration_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src);
Eigen::Matrix4f registration_ndt_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

//mapを元に算出
Eigen::Matrix4f map_ndt_vis(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_src, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,nav_msgs::Odometry odo);

#endif


