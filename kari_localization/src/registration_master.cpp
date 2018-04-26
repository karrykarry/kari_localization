#include <stdio.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>

#include "registration.hpp"

int main (int argc, char** argv)
{

	double l_roll, l_pitch, l_yaw;//角度etc

	ros::init(argc, argv, "registration_master");
	ros::NodeHandle n;
	ros::Rate roop(1);


	ros::Publisher trg_pub  = n.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1);
	ros::Publisher ipt_pub = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1);
	ros::Publisher out_pub = n.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1);

	// Loading first scan of room.
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);


	char file_input[100] = "/home/amsl/3Dmap/d_kan/clouds/cloud_1.pcd";
	char file_target[100] = "/home/amsl/3Dmap/d_kan/clouds/cloud_25.pcd";

	pcl::io::loadPCDFile(file_input, *input_cloud);
	pcl::io::loadPCDFile(file_target, *target_cloud);

    //
	// if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/amsl/3Dmap/d_kan/clouds/cloud_1.pcd", *target_cloud) == -1)
	// {
	// 	PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
	// 	return (-1);
	// }
	// std::cout << "Loaded " << target_cloud->size () << " data points from room_scan1.pcd" << std::endl;
    //
	// if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/amsl/3Dmap/d_kan/clouds/cloud_30.pcd", *input_cloud) == -1)
	// {
	// 	PCL_ERROR ("Couldn't read file room_scan2.pcd \n");
	// 	return (-1);
	// }
	// std::cout << "Loaded " << input_cloud->size () << " data points from room_scan2.pcd" << std::endl;
    //
    //


	tf::TransformBroadcaster br;
	tf::Transform transform;

	Eigen::Matrix4f a;

	a = registration_ndt_vis(target_cloud,input_cloud,output_cloud);

	cout<<"ndt"<<endl;

	// a = ndt.getFinalTransformation ();////4×4の行列         
	//最適の角度・長さを検出

	tf::Matrix3x3 mat_l;
	mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
			static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
			static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));

	mat_l.getRPY(l_roll, l_pitch, l_yaw, 1);

	//////////////////変換行列から得た答え           

	cout<<"t(0,3) : "<<a(0, 3)<<endl;
	cout<<"t(1,3) : "<<a(1, 3)<<endl;
	cout<<"l_roll : "<<l_roll<<endl;
	cout<<"l_pitch : "<<l_pitch<<endl;
	cout<<"l_yaw : "<<l_yaw<<endl;




	sensor_msgs::PointCloud2 pc, pc2, pc3;
	pcl::toROSMsg(*target_cloud, pc);
	pcl::toROSMsg(*input_cloud, pc2);
	pcl::toROSMsg(*output_cloud, pc3);


	while(ros::ok()){

		pc.header.frame_id  = "map";
		pc2.header.frame_id = "map";
		pc3.header.frame_id = "map";


		pc.header.stamp  = ros::Time::now();
		pc2.header.stamp = ros::Time::now();
		pc3.header.stamp = ros::Time::now();


		trg_pub.publish(pc);
		ipt_pub.publish(pc2);
		out_pub.publish(pc3);

		roop.sleep();
		ros::spinOnce();
	}

	return (0);
}


