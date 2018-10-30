//
//estimate_result で使いたい値を選択
//
//gyro:imu
//
#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

using namespace std;	

class Complement{
	private:
		ros::NodeHandle n;
		ros::Rate r;
		ros::Publisher lcl_pub;
		ros::Publisher lcl_vis_pub;

		tf::TransformBroadcaster br;
		tf::Transform transform; //位置と姿勢を持つ座標系を表すクラス

		geometry_msgs::Pose2D init_pose; // [deg] This expresses a position and orientation on a 2D manifold.
		nav_msgs::Odometry lcl_;
		nav_msgs::Odometry lcl_vis;

		bool start_flag;

		geometry_msgs::Quaternion odom_quat;
		double odom_vel;
		double dyaw;
		ros::Time current_time;
		ros::Time last_time;
		double x;
		double y;
		double yaw;
		//param
		string HEADER_FRAME;
		string CHILD_FRAME;
		string ODOM_TOPIC;
		string IMU_TOPIC;
		double drift_dyaw;

	public:
		Complement(ros::NodeHandle& n);
		void odomCallback(const nav_msgs::Odometry::Ptr msg);
		void imuCallback(const sensor_msgs::Imu::Ptr msg);
		void prepare();
		void start();
		void calc();
		bool spin()
		{
				
			ros::Rate loop_rate(r);
			
			while(ros::ok()){

				if(start_flag) start();

				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}

};




Complement::Complement(ros::NodeHandle &n)
	: r(50), start_flag(false)
{

	n.param("header_frame" ,HEADER_FRAME, {0});
	n.param("child_frame" ,CHILD_FRAME, {0});
	n.param("init_x" ,init_pose.x, 0.0);
	n.param("init_y" ,init_pose.y, 0.0);
	n.param("init_yaw" ,init_pose.theta, 0.0);//degree
	n.param("odom_topic" ,ODOM_TOPIC, {0});
	n.param("imu_topic" ,IMU_TOPIC, {0});
	n.param("dyaw/drift",drift_dyaw,{0});

	ros::Subscriber odm_sub = n.subscribe(ODOM_TOPIC, 100, &Complement::odomCallback, this);
	ros::Subscriber amu_sub = n.subscribe(IMU_TOPIC, 100, &Complement::imuCallback, this);

	lcl_pub = n.advertise<nav_msgs::Odometry>("lcl_ekf", 10);
	lcl_vis_pub = n.advertise<nav_msgs::Odometry>("/lcl_vis", 10);

	lcl_.header.frame_id = HEADER_FRAME;
	lcl_.child_frame_id = CHILD_FRAME;
	lcl_vis.header.frame_id = HEADER_FRAME;
	lcl_vis.child_frame_id = CHILD_FRAME;

	prepare();
}



void 
Complement::odomCallback(const nav_msgs::Odometry::Ptr msg){

	odom_vel = msg->twist.twist.linear.x;
	
}


void 
Complement::imuCallback(const sensor_msgs::Imu::Ptr imu){

	dyaw = imu->angular_velocity.z;
	dyaw -= drift_dyaw;					//tkhsh_imuの方で調節
	current_time = imu->header.stamp;
	if(!start_flag) last_time = current_time;
	start_flag = true;

	calc();
}


void
Complement::prepare(){

	transform.setOrigin( tf::Vector3(init_pose.x, init_pose.y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, init_pose.theta*M_PI/180.0);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), HEADER_FRAME, CHILD_FRAME));
}

void
Complement::calc(){
	
	double last_accurate,current_accurate;
	last_accurate = (double)last_time.nsec*1.0e-9 + last_time.sec;
	current_accurate = (double)current_time.nsec*1.0e-9 + current_time.sec;
	double dt = current_accurate - last_accurate;
	last_time = current_time;

	double dist = odom_vel * dt; 

	yaw += dyaw * dt; 

	while(yaw > M_PI) yaw -= 2*M_PI;
	while(yaw < -M_PI) yaw += 2*M_PI;

	x += dist * cos(yaw);// * cos(pitch);
	y += dist * sin(yaw);// * cos(pitch);
	odom_quat = tf::createQuaternionMsgFromYaw(yaw);
	
}




void
Complement::start(){

	lcl_.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている
	lcl_.pose.pose.position.x = x;
	lcl_.pose.pose.position.y = y;
	lcl_.pose.pose.position.z = 0.0;
	lcl_.pose.pose.orientation.z = yaw;

	lcl_pub.publish(lcl_);


	lcl_vis.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている
	lcl_vis.pose.pose.position.x = x;
	lcl_vis.pose.pose.position.y = y;
	lcl_vis.pose.pose.position.z = 0.0;	
	lcl_vis.pose.pose.orientation = odom_quat;

	lcl_vis_pub.publish(lcl_vis);


	transform.setOrigin( tf::Vector3( x, y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, yaw);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , HEADER_FRAME, CHILD_FRAME));

}


int main (int argc, char** argv){
	ros::init(argc,argv,"imu_complement");
	ros::NodeHandle n;

	cout<<"------complement start---------"<<endl;
	Complement complement(n);
	complement.spin();

	return 0;
}
