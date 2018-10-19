#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <pcl_conversions/pcl_conversions.h>

#include <local_tool/filters.hpp>

#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))

int grid_dim_ = 20;
double m_per_cell_ = 0.1;

using namespace std;	

const float map_limit = 1.0;

float x_now = 0;
float y_now = 0;

pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud (new pcl::PointCloud<pcl::PointXYZ>);

void lcl_Callback(const nav_msgs::OdometryConstPtr msg)//現在地把握
{
    x_now = msg->pose.pose.position.x;
    y_now = msg->pose.pose.position.y;

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


float get_min(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point){
       
    float min[grid_dim_][grid_dim_];
    bool init[grid_dim_][grid_dim_];
	
	fill(min[0],min[grid_dim_],0);
	fill(init[0],init[grid_dim_],0);

	float ans;
	int count;
	count =0;

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
	
	for (int i=0;i<grid_dim_;i++){
		for (int j=0;j<grid_dim_;j++){
			if(init[i][j]){
				ans = (ans * count + min[i][j])/(count+1);
				count++;
			}
		}
	}
	return ans;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "height_min2");
  	ros::NodeHandle n;
	ros::Rate roop(10);

	ros::Publisher pub_height  = n.advertise<std_msgs::Float64>("/height", 10);	
  
    ros::Subscriber odom_sub = n.subscribe("/lcl_ekf", 1000, lcl_Callback);//lclから現在地求む

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_IN(new pcl::PointCloud<pcl::PointXYZ>);
    
	string map_file;
	n.getParam("map/d_kan_around",map_file);
	map_reader(0.2,map_file,cloud_IN);
    
	std_msgs::Float64 num;
	
	while(ros::ok()){

		local_map(cloud_IN);	
        num.data = get_min(local_map_cloud);

		pub_height.publish(num);

		roop.sleep();
		ros::spinOnce();
	}
	return (0);
}
