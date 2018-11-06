#ifndef _PC2TRANSFORM4MAP_MAIN_HPP_
#define _PC2TRANSFORM4MAP_MAIN_HPP_

#include <ros/ros.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <local_tool/filters.hpp>


using namespace std;	


class 4Map_main
{
	private:
		float map_limit;
		int grid_dim_;
		double m_per_cell_;

	public:
		4Map_main();

};

#endif

