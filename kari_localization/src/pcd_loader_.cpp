//pcd_loader.cpp
//

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

using namespace std;	

int main (int argc, char** argv)
{
	ros::init(argc, argv, "pcd_loader_");
	ros::NodeHandle n;
	ros::Rate roop(1);

	string str;

	n.getParam("map_file", str);

	ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2>("/cloud_pcd", 100, true);

	pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_IN (new pcl::PointCloud<pcl::PointXYZINormal>);

	// if( pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(argv[1], cloud_IN2) == -1 ){
	if( pcl::io::loadPCDFile<pcl::PointXYZINormal>(str, *cloud_IN) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}

	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*cloud_IN, pc);

	pc.header.frame_id  = "map";	
	pc.header.stamp  = ros::Time::now();
	pub.publish(pc);

	sleep(1.0);
	return (0);
}
