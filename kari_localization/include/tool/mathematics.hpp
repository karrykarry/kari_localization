#ifndef _MATHEMATICS_HPP_
#define _MATHEMATICS_HPP_

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

void calc_rpy(Eigen::Matrix4f a,double &roll,double &pitch,double &yaw);
float distance(float x,float y);

#endif


