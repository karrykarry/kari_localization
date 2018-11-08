/* map_alignmennt_add_height.cpp:mapの地面に対して合わせてmap-matching
 *
 * 2018.11.02
 *
 * author : R.Kusakari
 *
 * subscribe lcl_ekf
 *
*/ 
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
CloudXPtr use_map_cloud (new pcl::PointCloud<PointX>);
CloudXPtr filtered_map_cloud2 (new pcl::PointCloud<PointX>);
CloudXPtr output_cloud (new pcl::PointCloud<PointX>);


class Align{
	private:
		ros::NodeHandle n;
		ros::Rate r;
		ros::Subscriber velo_sub;
		ros::Subscriber odom_sub;
		ros::Subscriber z_sub;
		ros::Subscriber wp_sub;
		ros::Publisher ndt_pub;
		ros::Publisher vis_voxel_pub;
		ros::Publisher vis_map_pub;

		sensor_msgs::PointCloud2 vis_laser_voxel;
		sensor_msgs::PointCloud2 vis_map;	
		nav_msgs::Odometry odo;
		nav_msgs::Odometry odo_pub;

		float x_now,y_now,yaw_now;
		float z_now;
		double l_roll, l_pitch, l_yaw;//角度etc

		bool flag;//lidar
		int map_mode;//lidar
		float laser_size,map_size;//voxel
		float map_limit,laser_limit;//map・laserの距離
		string LIDAR_TOPIC;
		string map_file;
		string map_file2;

	public:
		Align(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void map_read();//mapの点群を扱う
		void maptolidar();//位置合わせを行う

		void local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point);//mapを部分的に参照する

		void z_lcl_Callback(std_msgs::Float64 number);
		void odomCallback(const nav_msgs::Odometry::Ptr msg);
		void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr input);//velodyneの点群を扱う
		void wpCallback(const std_msgs::Int32ConstPtr input);//mapの切り替え
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

Align::Align(ros::NodeHandle n,ros::NodeHandle priv_nh) :
	r(10),flag(false)
{

    priv_nh.getParam("laser_voxel_size",laser_size);
    priv_nh.getParam("map_voxel_size",map_size);
    priv_nh.getParam("map_limit",map_limit);
    priv_nh.getParam("laser_limit",laser_limit);
    priv_nh.getParam("lidar_topic",LIDAR_TOPIC);
	n.getParam("map_file",map_file);
	// n.getParam("map_file2",map_file2);
	n.getParam("map_mode",map_mode);
		
	velo_sub = n.subscribe(LIDAR_TOPIC, 1000, &Align::velodyneCallback, this);
	odom_sub = n.subscribe("/lcl_ekf", 1000, &Align::odomCallback, this);
	z_sub = n.subscribe("/height", 1000, &Align::z_lcl_Callback, this);
	wp_sub = n.subscribe("/waypoint/now", 1000, &Align::wpCallback, this);

	ndt_pub = n.advertise<nav_msgs::Odometry>("/sq_ndt_data", 1000);
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

	yaw_now = msg->pose.pose.orientation.z;
	flag = true;
}


void 
Align::z_lcl_Callback(std_msgs::Float64 number){

	z_now = number.data + 0.7;

}

bool flag_ = true;
void 
Align::wpCallback(const std_msgs::Int32ConstPtr input){

	if(map_mode == 1){
	
	//d_kan_indoor
	if(input->data == 2){
		if(flag_){
		use_map_cloud = filtered_map_cloud2;
		flag_ = false;
		}
	}

	//d_kan_around
	else if(input->data == 13){
		if(flag_){
		use_map_cloud = filtered_map_cloud;
		flag_ = false;
		}
	}

	else flag_ = true;

	}


	if(map_mode == 2){
	//d_kan_around
	if(input->data == 2 || input->data == 15 ){
		if(flag_){
		use_map_cloud = filtered_map_cloud2;
		flag_ = false;
		}
	}

	//d_kan_indoor
	else if(input->data == 12 || input->data == 23){
		if(flag_){
		use_map_cloud = filtered_map_cloud;
		flag_ = false;
		}
	}


	else flag_ = true;
	}

	else{

		// if(input->data == 37){
		// 	if(flag_){
		// 		cout<< "change map" <<endl;
		// 		use_map_cloud = filtered_map_cloud2;
		// 		flag_ = false;
		// 	}
		// }

	}
}



//lidar情報
void 
Align::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr input){

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
        
        // if(-0.3 + z_now < temp_point.z && temp_point.z <4.0 + z_now){//つくば
		 if((0 <= d &&d <= laser_limit)) {//つくば
        // if(1.5 <= d &&d <= laser_limit){//つくば

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
	// map_reader(map_size,map_file2,filtered_map_cloud2);
	// cout << "mapから得た Filtered cloud contains " << filtered_map_cloud->size ()<< endl;
	use_map_cloud = filtered_map_cloud;
}




//mapの一部を算出
void 
Align::local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point)
{
    //limit_laserを設定
    float d = 0;
	local_map_cloud->points.clear();
	
    size_t velodyne_size = input_point->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = input_point->points[i].x; 
		temp_point.y = input_point->points[i].y;
		temp_point.z = input_point->points[i].z;

        
        d = distance(temp_point.x , temp_point.y);
		if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now)){//つくば
        
		// if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now) && (-0.5 + z_now  < temp_point.z && temp_point.z <= 5.0 + z_now )){//つくば
	 
			local_map_cloud->points.push_back(temp_point);
		}
	}
}





///メイン処理
void 
Align::maptolidar()
{
	// cout << "レーザから得た Filtered cloud contains " << filtered_laser_cloud->size ()<< endl;

	// local_map(filtered_map_cloud);
	local_map(use_map_cloud);

	Matrix4f a;
	a = map_ndt_vis(local_map_cloud,filtered_laser_cloud,output_cloud,odo);
	//a = map_icp_vis(local_map_cloud,filtered_laser_cloud,output_cloud,odo);

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
	ros::NodeHandle priv_nh("~");

	cout<<"-----alignment start-------"<<endl;

	Align align(n,priv_nh);

	align.spin();

	return 0;
}
