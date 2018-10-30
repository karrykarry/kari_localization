/* vis_laser.cpp
 *
 * publisher,subscriber,header_frame を指定後 lidarの点群を表示
 *
*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Odometry.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <iostream>


#include <local_tool/mathematics.hpp>

typedef pcl::PointXYZI PointX;
typedef pcl::PointCloud<PointX>::Ptr CloudXPtr;

CloudXPtr input_cloud (new pcl::PointCloud<PointX>);
CloudXPtr limit_input_cloud (new pcl::PointCloud<PointX>);

using namespace std;

class Laser
{
	private:
		ros::NodeHandle n;
		ros::Rate r;
		ros::Subscriber laser_sub;
		ros::Publisher laser_pub;

		string HEADER_FRAME;
		string LASER_SUB;
		string LASER_PUB;

	public:
		Laser(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void laserCallback(const sensor_msgs::PointCloud2Ptr msg);

};

Laser::Laser(ros::NodeHandle n,ros::NodeHandle priv_nh):
	r(10)
{

	priv_nh.getParam("header_frame",HEADER_FRAME);
	priv_nh.getParam("laser_sub",LASER_SUB);
	priv_nh.getParam("laser_pub",LASER_PUB);

	laser_sub = n.subscribe(LASER_SUB, 10, &Laser::laserCallback, this);
	laser_pub = n.advertise<sensor_msgs::PointCloud2>(LASER_PUB, 10);
}


void
Laser::laserCallback(const sensor_msgs::PointCloud2Ptr msg){
    sensor_msgs::PointCloud2 velo2;

     pcl::fromROSMsg(*msg,*input_cloud);
 
     limit_input_cloud->points.clear();
 
     size_t velodyne_size = input_cloud->points.size();
     for(size_t i = 0; i < velodyne_size; i++){
         pcl::PointXYZI temp_point;
         temp_point.x = input_cloud->points[i].x; 
         temp_point.y = input_cloud->points[i].y;
         temp_point.z = input_cloud->points[i].z;
         temp_point.intensity = input_cloud->points[i].intensity;
      
		 limit_input_cloud->points.push_back(temp_point);
     }

     pcl::toROSMsg(*limit_input_cloud,velo2);
    
	 // velo2.header.stamp = msg->header.stamp;
	 velo2.header.stamp =  ros::Time::now();
	 velo2.header.frame_id = HEADER_FRAME;

    laser_pub.publish(velo2);
}

int main(int argc, char** argv){
	ros::init(argc, argv, "vis_laser");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"-------vis laser ok--------"<<endl;

	Laser laser(n,priv_nh);

    ros::spin();
	
	return 0;
}


