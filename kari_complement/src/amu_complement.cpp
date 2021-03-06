//
//estimate_result で使いたい値を選択
//
//gyro:amu
//
//
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <ceres_msgs/AMU_data.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include "std_msgs/Int32.h"

#include <complement/localization.h>

using namespace std;	

// 0:odom 1:imu 2:ndt 3:ekf
const size_t N = 4;

class Complement{
	private:
		ros::NodeHandle n;
		ros::Rate r;
		ros::Subscriber odm_sub;
		ros::Subscriber amu_sub;
		ros::Subscriber ndt_sub;
		ros::Subscriber ekf_sub;

		ros::Publisher lcl_pub;
		ros::Publisher lcl_vis_pub;
		ros::Publisher lcl_hantei_pub;//0:直線 1:カーブ

		tf::TransformBroadcaster br;
		tf::Transform transform; //位置と姿勢を持つ座標系を表すクラス

		Localization lcl[N];

		geometry_msgs::Pose2D init_pose; // [deg] This expresses a position and orientation on a 2D manifold.
		nav_msgs::Odometry lcl_;
		nav_msgs::Odometry lcl_vis;

		sensor_msgs::Imu imu_msg;
		std_msgs::Int32 number;

		tfScalar yaw_first;

		bool flag;
		bool yaw_first_flag;
		bool w_first_flag;

		double w_first;
		double yaw_before;
		int num;

		string HEADER_FRAME;
		string CHILD_FRAME;
		string ODOM_TOPIC;
		string IMU_TOPIC;
		string NDT_TOPIC;
		string EKF_TOPIC;
		string PUB_TOPIC;
		int type;
	public:
		Complement(ros::NodeHandle& n);
		void odomCallback(const nav_msgs::Odometry::Ptr msg);
		void amuCallback(const ceres_msgs::AMU_data::Ptr msg);
		void ndtCallback(const nav_msgs::Odometry msg);
		void ekfCallback(const nav_msgs::Odometry msg);

		void prepare();
		void start();

		bool spin()
		{
				
			ros::Rate loop_rate(r);
			
			while(ros::ok()){
				if(flag)prepare();
				else start();

				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}

};




Complement::Complement(ros::NodeHandle &n) :
	r(100),
	flag(true),
	yaw_first_flag(true),
	w_first_flag(true)
{

	n.param("header_frame" ,HEADER_FRAME, {0});
	n.param("child_frame" ,CHILD_FRAME, {0});
	n.param("init_x" ,init_pose.x, 0.0);
	n.param("init_y" ,init_pose.y, 0.0);
	n.param("init_yaw" ,init_pose.theta, 0.0);//degree
	n.param("odom_topic" ,ODOM_TOPIC, {0});
	n.param("imu_topic" ,IMU_TOPIC, {0});
	n.param("ndt_topic" ,NDT_TOPIC, {0});
	n.param("ekf_topic" ,EKF_TOPIC, {0});
	n.param("publish_topic" ,PUB_TOPIC, {0});
	n.param("estimate_result" ,type, 0);//choice



	for(size_t i=0;i<N;i++){
		lcl[i].x =   init_pose.x;
		lcl[i].y =   init_pose.y;
		lcl[i].yaw = init_pose.theta*M_PI/180.0;
	}
	
	for(size_t i=0;i<N;i++) lcl[i].start();

	odm_sub = n.subscribe(ODOM_TOPIC, 100, &Complement::odomCallback, this);
	amu_sub = n.subscribe(IMU_TOPIC, 100, &Complement::amuCallback, this);
	ndt_sub = n.subscribe(NDT_TOPIC, 100, &Complement::ndtCallback, this);
    ekf_sub = n.subscribe(EKF_TOPIC, 100, &Complement::ekfCallback, this);

	lcl_pub = n.advertise<nav_msgs::Odometry>(PUB_TOPIC, 10);
	lcl_vis_pub = n.advertise<nav_msgs::Odometry>("/lcl_vis", 10);
	lcl_hantei_pub = n.advertise<std_msgs::Int32>("/hantei", 10);

	lcl_.header.frame_id = HEADER_FRAME;
	lcl_.child_frame_id = CHILD_FRAME;
	lcl_vis.header.frame_id = HEADER_FRAME;
	lcl_vis.child_frame_id = CHILD_FRAME;

	try{
		if(type == 0)cout<<"odom_result"<<endl;
		else if(type == 1)cout<<"amu_result"<<endl;
		else if(type == 2)cout<<"ndt_result"<<endl;
		else if(type == 3)cout<<"ekf_result"<<endl;
		else throw "error:This paramer is not available";
	}
	catch(char *errstr){
		cout<< errstr << endl;

	}


	num = 0;

}

void 
Complement::odomCallback(const nav_msgs::Odometry::Ptr msg){

	fflush(stdout);
	
    lcl[0].v = msg->twist.twist.linear.x;
	lcl[1].v = msg->twist.twist.linear.x;
	
    if(w_first_flag){	
		w_first = msg->twist.twist.angular.z;

        w_first_flag = false;
    }
	
    lcl[0].w = msg->twist.twist.angular.z - w_first;


    lcl[0].altering();
	// lcl[0].altering4(); //pitchが変だから
    flag=false;
}


void 
Complement::amuCallback(const ceres_msgs::AMU_data::Ptr msg){	

    fflush(stdout);
	
	lcl[0].pitch = M_PI * msg->pitch / 180.0;
	lcl[1].pitch = M_PI * msg->pitch / 180.0;

	lcl[1].w = - M_PI * (msg->dyaw - 0.273163) / 180.0;

	lcl[1].altering();

    // cout <<"dyaw"<<lcl[1].w<<endl;
 
    //num 10でekfのsigmaを変更 
    if(lcl[1].w < -0.001 || 0.001 < lcl[1].w){ //小さいほうがndtが吹っ飛ぶ前に補正できる//1[deg] = 0.017[rad]
    
    num = 10;

    }

    else num = 0;
}


void 
Complement::ndtCallback(const nav_msgs::Odometry msg){
	
	fflush(stdout);
    
    lcl[2].x = msg.pose.pose.position.x;
	lcl[2].y = msg.pose.pose.position.y;
	lcl[2].yaw = msg.pose.pose.orientation.z;
	
}


void 
Complement::ekfCallback(const nav_msgs::Odometry msg){

	fflush(stdout);
    
    lcl[3].x = msg.pose.pose.position.x;
	lcl[3].y = msg.pose.pose.position.y;
	lcl[3].yaw = msg.pose.pose.orientation.z;
	
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
Complement::start(){

// 0:odom 1:imu 2:ndt 3:ekf


	double u;

	// lcl_.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている
	lcl_.header.stamp = ros::Time(0); //timestampのメッセージを送ろうとしている
	lcl_.pose.pose.position.x = lcl[type].x;
	lcl_.pose.pose.position.y = lcl[type].y;
	lcl_.pose.pose.position.z = 0.0;
	lcl_.pose.pose.orientation.z = lcl[type].yaw;

	lcl_pub.publish(lcl_);

	u = (double)lcl[type].yaw;


	// lcl_vis.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている
	lcl_vis.header.stamp = ros::Time(0); //timestampのメッセージを送ろうとしている
	lcl_vis.pose.pose.position.x = lcl[type].x;
	lcl_vis.pose.pose.position.y = lcl[type].y;
	lcl_vis.pose.pose.position.z = 0.0;	
	lcl_vis.pose.pose.orientation = tf::createQuaternionMsgFromYaw(u) ;

	lcl_vis_pub.publish(lcl_vis);


	number.data = num;

	lcl_hantei_pub.publish(number);//0:直線 1:カーブ


	transform.setOrigin( tf::Vector3(lcl[type].x, lcl[type].y, 0.0) );
	tf::Quaternion q;
	q.setRPY(0, 0, lcl[type].yaw);

	transform.setRotation(q);
	// br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , HEADER_FRAME, CHILD_FRAME));
	br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , HEADER_FRAME, CHILD_FRAME));

}


int main (int argc, char** argv){
	ros::init(argc,argv,"amu_complement");
	ros::NodeHandle n;

	cout<<"------complement start---------"<<endl;
	Complement complement(n);
	complement.spin();

	return 0;
}

