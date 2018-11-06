/*自分付近の最下点を算出
 *
 */
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
#include <std_msgs/Int32.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

#include <local_tool/filters.hpp>

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

int grid_dim_ = 8;
double m_per_cell_ = 0.5;

using namespace std;	

const float map_limit = 2.0;

float x_now = 0;
float y_now = 0;
float yaw_now = 0;

bool map_mode;


sensor_msgs::PointCloud2 transformed_pc;

pcl::PointCloud<pcl::PointXYZ>::Ptr use_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_IN(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_IN2(new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr before_cloud (new pcl::PointCloud<pcl::PointXYZ>);

struct Data {
	float min;
	float max;
	int min_d;
	int max_d;
};


void lcl_Callback(const nav_msgs::OdometryConstPtr msg)//現在地把握
{
    x_now = msg->pose.pose.position.x;
    y_now = msg->pose.pose.position.y;
    yaw_now = msg->pose.pose.orientation.z;

}

void pcCallback(const sensor_msgs::PointCloud2ConstPtr msg)//現在地把握
{
	pcl::fromROSMsg(*msg,*before_cloud);
}

void local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point)
{
    //limit_laserを設定
	local_map_cloud->points.clear();
	
    size_t velodyne_size = input_point->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = input_point->points[i].x; 
		temp_point.y = input_point->points[i].y;
		temp_point.z = input_point->points[i].z;

         
        //if(d <= map_limit && (-0.5 < temp_point.z && temp_point.z <5.0)){
		if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now)) {//つくば
	
			local_map_cloud->points.push_back(temp_point);
		}
	}
}


void get_min(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point,float& z_ans,float& pitch_ans,float& roll_ans){
       
    float min[grid_dim_][grid_dim_];
    bool init[grid_dim_][grid_dim_];
	
	fill(min[0],min[grid_dim_],100);
	fill(init[0],init[grid_dim_],0);

	float pitch[grid_dim_] = {};
	float roll[grid_dim_] = {};

	Data pitch_;
	Data roll_;

	int count;
	z_ans =0;
	pitch_ans =0;
	roll_ans =0;

    size_t velodyne_size = input_point->points.size();
    for (size_t i = 0; i < velodyne_size; ++i) {
        pcl::PointXYZ scan;
        
        scan.x = input_point->points[i].x - x_now;
        scan.y = input_point->points[i].y - y_now;
        scan.z = input_point->points[i].z;

        int x = ((grid_dim_/2)+scan.x /m_per_cell_);
        int y = ((grid_dim_/2)+scan.y /m_per_cell_);

        if (x >= 0 && x < grid_dim_  && y >= 0 && y < grid_dim_) {
			if (!init[x][y]) {
                min[x][y] = scan.z;
                // all_grid++;
                init[x][y] = true;
            } 
			else min[x][y] = MIN(min[x][y], scan.z);
        }
    }

	count =0;
	// tkb用
	//minimumの近傍を平均する
	//roll
	for (int i=0;i<grid_dim_;i++){
		bool flag = false;
		roll_.max_d = 0;
		
		for (int j=0;j<grid_dim_;j++){
			if(init[i][j]){
				z_ans = (z_ans * count + min[i][j])/(count+1);
				count++;
				
				if(!flag){
					roll_.min = min[i][j];
					roll_.min_d = j;
					flag = true;
				}
				roll_.max = min[i][j];	
				roll_.max_d = j;
			}	
		}	
		if(roll_.max_d) roll[i] = atan2((-1)*(roll_.max - roll_.min),(roll_.max_d-roll_.min_d+1)*m_per_cell_ );
	}
	
	//pitch
	for (int i=0;i<grid_dim_;i++){
		bool flag = false;
		pitch_.max_d = 0;
		
		for (int j=0;j<grid_dim_;j++){
			if(init[j][i]){
				if(!flag){
					pitch_.min = min[j][i];
					pitch_.min_d = j;
					flag = true;
				}
				pitch_.max = min[j][i];	
				pitch_.max_d = j;
			}	
		}	
		if(pitch_.max_d) pitch[i] = atan2((-1)*(pitch_.max - pitch_.min),(pitch_.max_d-pitch_.min_d+1)*m_per_cell_);
	}

	count =0;

	for(int i=0;i<grid_dim_;i++){
		// cout<<pitch[i]<<endl;
		if(pitch[i]){
			pitch_ans = (pitch_ans * count + pitch[i])/(count+1);
			count++;
		}
	}

	count =0;

	for(int i=0;i<grid_dim_;i++){
		// cout<<roll[i]<<endl;
		if(roll[i]){
			roll_ans = (roll_ans * count + roll[i])/(count+1);
			count++;
		}
	}
	// cout<<"roll:"<<roll_ans<<"pitch:"<<pitch_ans<<endl;

}


void pc2transform(float pitch_theta,float roll_theta){
	
	// float yaw_theta = 0;
    //
	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
	//
	// transform_1 (0,0) = cos (pitch_theta) * cos(yaw_theta);
	// transform_1 (0,1) = (-1) * sin(roll_theta) * (-1) * sin(pitch_theta) * cos(yaw_theta) - cos(roll_theta ) * sin(yaw_theta);
	// transform_1 (0,2) = cos(roll_theta) * (-1) * sin(pitch_theta) * cos(yaw_theta) + (-1) * sin(roll_theta ) * sin(yaw_theta);
	// transform_1 (1,0) = cos(pitch_theta) * sin(yaw_theta);
	// transform_1 (1,1) = (-1) * sin(roll_theta) * (-1) * sin(pitch_theta) * sin(yaw_theta) + cos(roll_theta ) * cos(yaw_theta);
	// transform_1 (1,2) = cos(roll_theta) * (-1) * sin(pitch_theta) * sin(yaw_theta) -(-1) * sin(roll_theta)  * cos(yaw_theta);
	// transform_1 (2,0) = (-1) * -sin (pitch_theta);
	// transform_1 (2,1) = (-1) * sin(roll_theta) * cos(pitch_theta);
	// transform_1 (2,2) = cos(roll_theta) * cos(pitch_theta);
	//
	// pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	// pcl::transformPointCloud (*before_cloud, *transformed_cloud, transform_1);
	// pcl::toROSMsg(*transformed_cloud,transformed_pc);


	Eigen::Matrix3f rot;
	rot = Eigen::AngleAxisf(roll_theta*(-1), Eigen::Vector3f::UnitX()) * Eigen::AngleAxisf(pitch_theta*(-1), Eigen::Vector3f::UnitY());

	Eigen::Translation3f init_translation (0, 0, 0);

	Eigen::Matrix4f transform = (rot * init_translation).matrix ();
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	pcl::transformPointCloud (*before_cloud, *transformed_cloud, transform);
	pcl::toROSMsg(*transformed_cloud,transformed_pc);






}

bool flag_ = true;
void wpCallback(const std_msgs::Int32ConstPtr input){

	if(map_mode == 1){
	
	//d_kan_indoor
	if(input->data == 3 ){
		if(flag_){
		use_map_cloud = cloud_IN2;
		flag_ = false;
		}
	}

	//d_kan_around
	else if(input->data == 13){
		if(flag_){
		use_map_cloud = cloud_IN;
		flag_ = false;
		}
	}

	else flag_ = true;

	}


	if(map_mode == 2){
	//d_kan_around
	if(input->data == 2 || input->data == 15 ){
		if(flag_){
		use_map_cloud = cloud_IN2;
		flag_ = false;
		}
	}

	//d_kan_indoor
	else if(input->data == 12 || input->data == 23){
		if(flag_){
		use_map_cloud = cloud_IN;
		flag_ = false;
		}
	}


	else flag_ = true;
	}

	else{

		if(input->data == 37){
			if(flag_){	
				cout<< "change map" <<endl;
				use_map_cloud = cloud_IN2;
				flag_ = false;
			}
		}

	}

}

// void tf_broad(double roll_now, double pitch_now, double yaw_now ,double x_now,double y_now,double z_now){
// 	tf::TransformBroadcaster br;
// 	tf::Transform transform; //位置と姿勢を持つ座標系を表すクラス
// 	
// 	
// 	transform.setOrigin( tf::Vector3(x_now, y_now, z_now) );
// 	tf::Quaternion q;
// 	q.setRPY(roll_now, pitch_now, yaw_now);
//
// 	transform.setRotation(q);
// 	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map" ,"sample_baselink"));
//
// 	cout<<"a"<<endl;
// }


int main (int argc, char** argv)
{
	ros::init(argc, argv, "pc2transform4map");
  	ros::NodeHandle n;
	ros::Rate roop(10);

	ros::Publisher pub_height  = n.advertise<std_msgs::Float64>("/height", 10);	
	ros::Publisher trans_pub  = n.advertise<sensor_msgs::PointCloud2>("/velodyne_obstacles_after", 10);	
  
    ros::Subscriber odom_sub = n.subscribe("/lcl_ekf", 1000, lcl_Callback);//lclから現在地求む
    ros::Subscriber pc_sub = n.subscribe("/velodyne_obstacles", 1000, pcCallback);
	ros::Subscriber wp_sub = n.subscribe("/waypoint/now", 1000, wpCallback);


	n.getParam("map_mode",map_mode);


	string map_file;
	string map_file2;
	n.getParam("map_file",map_file);
	n.getParam("map_file2",map_file2);
	map_reader(0.2,map_file,cloud_IN);
	map_reader(0.2,map_file2,cloud_IN2);
	
	use_map_cloud = cloud_IN;

	std_msgs::Float64 num;
	float z_ans;
	float pitch_ans;
	float roll_ans;
	
	while(ros::ok()){

		local_map(use_map_cloud);	
        get_min(local_map_cloud,z_ans,pitch_ans,roll_ans);

		num.data = z_ans;
		pub_height.publish(num);

		pc2transform(pitch_ans,roll_ans);

		transformed_pc.header.stamp = ros::Time::now();
		transformed_pc.header.frame_id = "/velodyne";
		
		trans_pub.publish(transformed_pc);

		// tf_broad(pitch_ans,roll_ans,yaw_now,x_now,y_now,z_ans);

		roop.sleep();
		ros::spinOnce();
	}
	return (0);
}
