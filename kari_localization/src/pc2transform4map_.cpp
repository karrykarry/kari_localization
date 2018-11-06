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

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <local_tool/filters.hpp>

#include "pc2transrform4map_main"


using namespace std;


class Transform4map
{
	private:
		ros::Rate r;
		ros::Subscriber obstacle_flag_sub;
		ros::Subscriber road_sub;
		ros::Publisher edge_pub;
		float x_now;
		float y_now;
		string map_file;
		4Map_main 4map_main;

	public:
		Transform4map(ros::NodeHandle n, ros::NodeHandle priv_nh);
		~Transform4map();

		void lclCallback(const nav_msg::OdometryConstPtr msg);
		void process();
		bool spin()
		{
			ros::Rate loop_rate(r);

			while(ros::ok()){
				process();
				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}
};

Transform4map::Transform4map(ros::NodeHandle n, ros::NodeHandle priv_nh)	:
	r(20),x_now(0.0),y_now(0.0)
{
	odom_sub = n.subscribe("/lcl_ekf", 10, &Transform4map::lclCallback, this);	
	height_pub = n.advertise<std_msgs::Float64>("height", 10);
	n.getParam("map_file",map_file);

}

Transform4map::~Transform4map()
{
}



void
Transform4map::lclCallback(const nav_msg::OdometryConstPtr msg){
	x_now = msg->pose.pose.position.x;
	y_now = msg->pose.pose.position.x;
}

void
Transform4map::process( ){
	num.data =  4map_main.getmin(4map_main.local_map(map_file))

}

int main(int argc, char** argv){
	ros::init(argc, argv, "pc2transrform4map");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	Transform4map transform4map(n,priv_nh);
	
	Transform4map.spin();

	return 0;
}

