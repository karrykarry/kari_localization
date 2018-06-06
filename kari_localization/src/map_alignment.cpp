//
//map_alignment.launch をつかってパラメータ調整する
//
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

#include <math.h>

#include <local_tool/filters.hpp>
#include <local_tool/mathematics.hpp>
#include <local_tool/registration.hpp>


typedef pcl::PointXYZINormal PointN;
typedef pcl::PointXYZ PointX;
typedef pcl::PointCloud<PointN>::Ptr CloudNPtr;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;

using namespace std;
using namespace Eigen;

CloudXPtr input_cloud (new pcl::PointCloud<PointX>);
CloudXPtr limit_lidar (new pcl::PointCloud<PointX>);
CloudXPtr limit_input_cloud (new pcl::PointCloud<PointX>);
CloudXPtr filtered_laser_cloud (new pcl::PointCloud<PointX>);
CloudXPtr local_map_cloud (new pcl::PointCloud<PointX>);
CloudXPtr target_map_cloud (new pcl::PointCloud<PointX>);
CloudXPtr filtered_map_cloud (new pcl::PointCloud<PointX>);
CloudXPtr output_cloud (new pcl::PointCloud<PointX>);


class Align{
	private:
		ros::NodeHandle n;
		ros::Rate r;
		ros::Subscriber velo_sub;
		ros::Subscriber odom_sub;
		ros::Publisher ndt_pub;
		ros::Publisher vis_voxel_pub;
		ros::Publisher vis_map_pub;

		sensor_msgs::PointCloud2 vis_laser_voxel;
		sensor_msgs::PointCloud2 vis_map;	
		nav_msgs::Odometry odo;
		nav_msgs::Odometry odo_pub;

		float x_now,y_now,yaw_now;
		float z_now;

		float map_limit,laser_limit;//map・laserの距離
		string map_file;

		float laser_size,map_size;//voxel
		bool flag;//lidar
		double l_roll, l_pitch, l_yaw;//角度etc


		string LIDAR_TOPIC;
		string ODOM_TOPIC;
		string NDT_TOPIC;

	public:
		Align(ros::NodeHandle& n);
		void map_read();//mapの点群を扱う
		void maptolidar();//位置合わせを行う

		void local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point);//mapを部分的に参照する

		void odomCallback(const nav_msgs::Odometry::Ptr msg);
		void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr input);//velodyneの点群を扱う
		bool spin()
		{
			ros::Rate loop_rate(r);

			map_read();
			while(ros::ok()){
				if(flag)maptolidar();
				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}
};

Align::Align(ros::NodeHandle& n) :
	r(10),
	flag(false)
{

    n.param("laser_voxel_size",laser_size, 0.0f);
    n.param("map_voxel_size",map_size, 0.0f);
    n.param("map_limit",map_limit, 0.0f);
    n.param("laser_limit",laser_limit, 0.0f);
    n.param("init_x",x_now, 0.0f);
    n.param("init_y",y_now, 0.0f);
    n.param("init_z",z_now, 0.0f);
    n.param("init_yaw",yaw_now, 0.0f);
    n.param("lidar_topic",LIDAR_TOPIC, {0});
    n.param("odom_topic",ODOM_TOPIC, {0});
    n.param("ndt_topic",NDT_TOPIC, {0});
    // n.getParam("ndt_topic",NDT_TOPIC);
	n.getParam("map/d_kan_around",map_file);
	// n.getParam("map/d_kan_indoor",map_file);
	
	
	velo_sub = n.subscribe(LIDAR_TOPIC, 1000, &Align::velodyneCallback, this);
	odom_sub = n.subscribe(ODOM_TOPIC, 1000, &Align::odomCallback, this);

	ndt_pub = n.advertise<nav_msgs::Odometry>(NDT_TOPIC, 1000);
	vis_voxel_pub = n.advertise<sensor_msgs::PointCloud2>("/ndt_result", 1000);
	vis_map_pub = n.advertise<sensor_msgs::PointCloud2>("/vis_map", 1000);



	// cout << "-------PARAMETARS----------" << endl;
	cout << "map_name：" << map_file << endl;

	l_roll = l_pitch = l_yaw = 0;

}

//現在地を知る
void 
Align::odomCallback(const nav_msgs::Odometry::Ptr msg){

    odo.pose = msg->pose;

	x_now = msg->pose.pose.position.x;
	y_now = msg->pose.pose.position.y;
	z_now = 0;

	yaw_now = msg->pose.pose.orientation.z;
}

//lidar情報
void 
Align::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr input){

	flag = true;
	float d = 0;

	pcl::fromROSMsg(*input,*input_cloud);

	limit_input_cloud->points.clear();
	
    size_t velodyne_size = input_cloud->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = input_cloud->points[i].x; 
		temp_point.y = input_cloud->points[i].y;
		temp_point.z = input_cloud->points[i].z + z_now;
		
        d = distance(temp_point.x , temp_point.y);
        
        // if((1.5 <= d &&d <= laser_limit) && (-0.3 + z_now < temp_point.z && temp_point.z <4.0 + z_now)){//つくば
        if((1.5 <= d &&d <= laser_limit)&&(-0.3 + z_now < temp_point.z && temp_point.z <2.0 + z_now)) {//つくば

			limit_input_cloud->points.push_back(temp_point);
	
		}
	
	}

	voxel_grid(laser_size,limit_input_cloud,filtered_laser_cloud);

}


//参照データの読み込み
void 
Align::map_read()
{
	map_reader(map_size,map_file,filtered_map_cloud);
	cout << "mapから得た Filtered cloud contains " << filtered_map_cloud->size ()<< endl;

}


//mapの一部を算出
void 
Align::local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point)
{
    //limit_laserを設定
    float d = 0 ;
	local_map_cloud->points.clear();
	
    size_t velodyne_size = input_point->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = input_point->points[i].x; 
		temp_point.y = input_point->points[i].y;
		temp_point.z = input_point->points[i].z;

        
        d = distance(temp_point.x , temp_point.y);
        
		if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now) && (-0.5 + z_now  < temp_point.z && temp_point.z <= 5.0 + z_now )){//つくば
	 
			local_map_cloud->points.push_back(temp_point);
		}
	}
}





///メイン処理
void 
Align::maptolidar()
{
	cout << "レーザから得た Filtered cloud contains " << filtered_laser_cloud->size ()<< endl;

	local_map(filtered_map_cloud);

	Matrix4f a;
	a = map_ndt_vis(local_map_cloud,filtered_laser_cloud,output_cloud,odo);

	calc_rpy(a,l_roll,l_pitch,l_yaw);

	odo_pub.pose.pose.position.x =  a(0, 3); 
	odo_pub.pose.pose.position.y =  a(1, 3); 
	odo_pub.pose.pose.orientation.z = l_yaw;  

	ndt_pub.publish(odo_pub);               

	pcl::toROSMsg(*output_cloud , vis_laser_voxel);           
	pcl::toROSMsg(*local_map_cloud , vis_map);           


    vis_laser_voxel.header.frame_id = "/map"; //laserのframe_id
	vis_map.header.frame_id = "/map"; //laserのframe_id

	vis_laser_voxel.header.stamp = ros::Time::now(); //laserのframe_id
	vis_map.header.stamp = ros::Time::now(); //laserのframe_id


	vis_voxel_pub.publish(vis_laser_voxel);
	vis_map_pub.publish(vis_map);

}






int main(int argc, char** argv){
	ros::init(argc,argv,"map_alignment");
	ros::NodeHandle n;


	cout<<"-----alignment start-------"<<endl;

	Align align(n);

	align.spin();

	return 0;
}
