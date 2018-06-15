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
		ros::Publisher map_pub;

		pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud;

		string file_input;
		string file_target;

		string file;

	public:
		Regi(ros::NodeHandle& n);

		void alignment(void);
		void cloud_data(void);
		void map(void);

		bool spin()
		{
			ros::Rate loop_rate(r);
			cloud_data();

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
	r(1)
{
	trg_pub  = n.advertise<sensor_msgs::PointCloud2>("/target_cloud", 1);
	ipt_pub  = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1);
	out_pub  = n.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1);
	map_pub  = n.advertise<sensor_msgs::PointCloud2>("/map_cloud", 1);

	n.getParam("input/cloud",file_input);
	n.getParam("target/cloud",file_target);

	target_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
	input_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
	output_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);
	map_cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);


	file = "/home/amsl/onda_map/d_kan_around_si2017_gicp_ds.pcd";
	map_reader(0.5,file,map_cloud);

}

void
Regi::cloud_data (void){

	pcl::io::loadPCDFile(file_input, *input_cloud);
	pcl::io::loadPCDFile(file_target, *target_cloud);

}



void
Regi::map (void){

	sensor_msgs::PointCloud2 pc4;
	
	pcl::toROSMsg(*map_cloud,pc4);

	pc4.header.frame_id = "map";
	pc4.header.stamp  = ros::Time::now();
	map_pub.publish(pc4);

}


void
Regi::alignment(void){

	Eigen::Matrix4f a;
	double l_roll, l_pitch, l_yaw;//角度etc

	// a = registration_ndt_vis(target_cloud,input_cloud,output_cloud);
	a = registration_icp_vis(target_cloud,input_cloud,output_cloud);

	calc_rpy(a,l_roll,l_pitch,l_yaw);
	
	//////////////////変換行列から得た答え          
	//
	cout<<"t(0,3) : "<<a(0, 3)<<endl;
	cout<<"t(1,3) : "<<a(1, 3)<<endl;
	cout<<"t(2,3) : "<<a(1, 3)<<endl;
	cout<<"l_roll : "<<l_roll<<endl;
	cout<<"l_pitch : "<<l_pitch<<endl;
	cout<<"l_yaw : "<<l_yaw<<endl;

	sensor_msgs::PointCloud2 pc, pc2, pc3;
	pcl::toROSMsg(*target_cloud, pc);
	pcl::toROSMsg(*input_cloud, pc2);
	pcl::toROSMsg(*output_cloud, pc3);

	pc.header.frame_id  = "map";
	pc2.header.frame_id = "map";
	pc3.header.frame_id = "map";

	pc.header.stamp  = ros::Time::now();
	pc2.header.stamp = ros::Time::now();
	pc3.header.stamp = ros::Time::now();

	trg_pub.publish(pc);
	ipt_pub.publish(pc2);
	out_pub.publish(pc3);

}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "registration_master");
	ros::NodeHandle n;

	cout<<"-------registration start---------"<<endl;

	Regi regi(n);

	regi.spin();

	return (0);
}


