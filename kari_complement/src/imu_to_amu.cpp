#include <ros/ros.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/Imu.h>
#include <ceres_msgs/AMU_data.h>

using namespace std;

class ITA{
	private:
		ros::NodeHandle private_nh_;
		ros::Subscriber imu_sub;
		ros::Publisher amu_pub;

	public:
		ITA(ros::NodeHandle& n);
		void imucallback(const sensor_msgs::Imu::Ptr msg);
};

ITA::ITA(ros::NodeHandle& n):
	private_nh_("~")
{
	imu_sub = n.subscribe("/imu/data",10,&ITA::imucallback,this);
	amu_pub = n.advertise<ceres_msgs::AMU_data>("/AMU_data",10);

	cout<<"start handle"<<endl;
}

void ITA::imucallback(const sensor_msgs::Imu::Ptr msg)
{
//	cout<<"callback"<<endl;
	ceres_msgs::AMU_data amu;
	amu.header.stamp = msg->header.stamp;
	amu.xaccel = msg->linear_acceleration.x;
	amu.yaccel =- msg->linear_acceleration.y;
	amu.zaccel = msg->linear_acceleration.z;
	amu.droll = msg->angular_velocity.x*180.0/M_PI;
	amu.dpitch = msg->angular_velocity.y*180.0/M_PI;
	amu.dyaw = -msg->angular_velocity.z*180.0/M_PI;
	tf::Quaternion q(
			msg->orientation.x,
			msg->orientation.y,
			msg->orientation.z,
			msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll,pitch,yaw;
	m.getRPY(roll,pitch,yaw);
	amu.roll = roll*180.0/M_PI;
	amu.pitch = pitch*180.0/M_PI;
	amu.yaw = - yaw*180.0/M_PI;

	amu_pub.publish(amu);
}

int main(int argc,char** argv)
{
	ros::init(argc,argv,"imu_to_amu");
	ros::NodeHandle n("~");

	ITA ita(n);

	ros::spin();

	return 0;
}
