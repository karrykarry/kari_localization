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
		Laser(ros::NodeHandle& n);
		void laserCallback(const sensor_msgs::PointCloud2 msg);

};

Laser::Laser(ros::NodeHandle &n) :
	r(10)
{

	n.getParam("header_frame",HEADER_FRAME);
	n.getParam("laser_sub",LASER_SUB);
	n.getParam("laser_pub",LASER_PUB);

	laser_sub = n.subscribe(LASER_SUB, 10, &Laser::laserCallback, this);
	laser_pub = n.advertise<sensor_msgs::PointCloud2>(LASER_PUB, 10);
}


void
Laser::laserCallback(const sensor_msgs::PointCloud2 msg){
    sensor_msgs::PointCloud2 velo2;
    
    velo2 = msg;

    velo2.header.stamp = ros::Time::now();
    velo2.header.frame_id = HEADER_FRAME;

    laser_pub.publish(velo2);

}

int main(int argc, char** argv){
	ros::init(argc, argv, "vis_laser");
	ros::NodeHandle n;

	cout<<"-------vis laser ok--------"<<endl;

	Laser laser(n);

    ros::spin();
	
	return 0;
}


