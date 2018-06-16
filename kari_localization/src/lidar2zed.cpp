#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>


#include <local_tool/filters.hpp>
#include <local_tool/mathematics.hpp>
#include <local_tool/registration.hpp>


using namespace std;


class Regi
{
	private:
		ros::NodeHandle n;
		ros::Rate r;

		ros::Publisher trg_pub;
		ros::Publisher ipt_pub;
		ros::Publisher out_pub;

		ros::Subscriber lidar_sub;
		ros::Subscriber camera_sub;


		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud;

		pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr camera_cloud;


		string LIDAR_TOPIC;
		string CAMERA_TOPIC;
		
		string PARENT_FRAME;

	public:
		Regi(ros::NodeHandle& n);

		void alignment(void);
		void map(void);
		
		void lidarCallback(const sensor_msgs::PointCloud2ConstPtr msgs);
		void cameraCallback(const sensor_msgs::PointCloud2ConstPtr msgs);
		bool spin()
		{
			ros::Rate loop_rate(r);

			while(ros::ok()){
				alignment();
				// map();	//mapの確認
				ros::spinOnce();
				loop_rate.sleep();

			}
			return true;
		}

};

Regi::Regi(ros::NodeHandle &n) :
	r(7)
{

	n.getParam("lidar_topic",LIDAR_TOPIC);
	n.getParam("camera_topic",CAMERA_TOPIC);
	n.getParam("parent_frame",PARENT_FRAME);
	
	trg_pub  = n.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1);
	ipt_pub  = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1);
	out_pub  = n.advertise<sensor_msgs::PointCloud2>("/lidar2zed", 1);


	lidar_sub = n.subscribe(LIDAR_TOPIC,1000,&Regi::lidarCallback,this);
	camera_sub = n.subscribe(CAMERA_TOPIC,1000,&Regi::cameraCallback,this);


	target_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
	input_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
	output_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);

	lidar_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
	camera_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);

}


void
Regi::lidarCallback(const sensor_msgs::PointCloud2ConstPtr msgs){

	pcl::fromROSMsg(*msgs,*lidar_cloud);

}

void
Regi::cameraCallback(const sensor_msgs::PointCloud2ConstPtr msgs){

	pcl::fromROSMsg(*msgs,*camera_cloud);
}


void
Regi::alignment(void){

	Eigen::Matrix4f a;
	double l_roll, l_pitch, l_yaw;//角度etc


	//icpはinputをtargetに持っていく input = output
	// a = registration_icp_vis(target_cloud,input_cloud,output_cloud);
	// a = registration_ndt_vis(target_cloud,input_cloud,output_cloud);
	a = registration_icp_vis(lidar_cloud,camera_cloud,output_cloud);

	calc_rpy(a,l_roll,l_pitch,l_yaw);
	
	//////////////////変換行列から得た答え          
	//
	cout<<"t(0,3) : "<<a(0, 3)<<endl;
	cout<<"t(1,3) : "<<a(1, 3)<<endl;
	cout<<"t(2,3) : "<<a(2, 3)<<endl;
	cout<<"l_roll : "<<l_roll<<endl;
	cout<<"l_pitch : "<<l_pitch<<endl;
	cout<<"l_yaw : "<<l_yaw<<endl;

	sensor_msgs::PointCloud2 pc, pc2, pc3;
	pcl::toROSMsg(*camera_cloud, pc);
	pcl::toROSMsg(*lidar_cloud, pc2);
	pcl::toROSMsg(*output_cloud, pc3);

	pc.header.frame_id  = "odom";
	pc2.header.frame_id = "odom";
	pc3.header.frame_id = "centerlaser";

	pc.header.stamp  = ros::Time::now();
	pc2.header.stamp = ros::Time::now();
	pc3.header.stamp = ros::Time::now();

	trg_pub.publish(pc);
	ipt_pub.publish(pc2);
	out_pub.publish(pc3);

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "lidar2zed");
	ros::NodeHandle n;

	cout<<"-------lidar2zed start---------"<<endl;

	Regi regi(n);

	regi.spin();

	return (0);
}



