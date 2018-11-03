//rviz の 2D Nav Goalで初期位置設定
//
//estimate_result で使いたい値を選択
//
//
#include <stdio.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>

#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"

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
		ros::Subscriber height_sub;
		ros::Subscriber initpose_sub;

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

		int num;
		bool flag;
		double z_height;

		string HEADER_FRAME;
		string CHILD_FRAME;
		int type;
		double drift_dyaw;

	public:
		Complement(ros::NodeHandle n,ros::NodeHandle priv_nh);
		void odomCallback(const nav_msgs::Odometry::Ptr msg);
		void imuCallback(const sensor_msgs::Imu::Ptr msg);
		void ndtCallback(const nav_msgs::Odometry msg);
		void ekfCallback(const nav_msgs::Odometry msg);
		void heightCallback(const std_msgs::Float64::Ptr msg);
		void initposeCallback(const geometry_msgs::PoseStampedConstPtr& msg);

		void prepare();
		int start();

		bool spin()
		{
				
			ros::Rate loop_rate(r);
			
			while(ros::ok()){

				if(flag) start();

				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}

};




Complement::Complement(ros::NodeHandle n,ros::NodeHandle priv_nh) :
	r(100),num(0),flag(false),z_height(0.0)
{

	priv_nh.getParam("header_frame" ,HEADER_FRAME);
	priv_nh.getParam("child_frame" ,CHILD_FRAME);
	priv_nh.getParam("init_x" ,init_pose.x);
	priv_nh.getParam("init_y" ,init_pose.y);
	priv_nh.getParam("init_yaw" ,init_pose.theta);//rad
	priv_nh.getParam("estimate_result" ,type);//choice
	priv_nh.getParam("dyaw/drift",drift_dyaw);

	for(size_t i=0;i<N;i++){
		lcl[i].x =   init_pose.x;
		lcl[i].y =   init_pose.y;
		lcl[i].yaw = init_pose.theta;
	}

	odm_sub = n.subscribe("/odom", 100, &Complement::odomCallback, this);
	amu_sub = n.subscribe("/imu/data", 100, &Complement::imuCallback, this);
	ndt_sub = n.subscribe("/sq_ndt_data", 100, &Complement::ndtCallback, this);
    ekf_sub = n.subscribe("/ekf_NDT", 100, &Complement::ekfCallback, this);
	height_sub = n.subscribe("/height", 100, &Complement::heightCallback, this);
	initpose_sub = n.subscribe("/move_base_simple/goal", 100, &Complement::initposeCallback, this);

	lcl_pub = n.advertise<nav_msgs::Odometry>("lcl_ekf", 10);
	lcl_vis_pub = n.advertise<nav_msgs::Odometry>("/lcl_vis", 10);
	lcl_hantei_pub = n.advertise<std_msgs::Int32>("/hantei", 10);

	lcl_.header.frame_id = HEADER_FRAME;
	lcl_.child_frame_id = CHILD_FRAME;
	lcl_vis.header.frame_id = HEADER_FRAME;
	lcl_vis.child_frame_id = CHILD_FRAME;

	try{
		if(type == 0)cout<<"odom_result"<<endl;
		else if(type == 1)cout<<"imu_result"<<endl;
		else if(type == 2)cout<<"ndt_result"<<endl;
		else if(type == 3)cout<<"ekf_result"<<endl;
		else throw "error:This paramer is not available";
	}
	catch(char *errstr){
		cout<< errstr << endl;

	}


}

void 
Complement::heightCallback(const std_msgs::Float64::Ptr msg){
	z_height = msg->data;
}


void 
Complement::odomCallback(const nav_msgs::Odometry::Ptr msg){

	if(!lcl[0].flag){	
		lcl[0].start();
        lcl[0].flag = true;
    }
	
    lcl[0].v = msg->twist.twist.linear.x;
	lcl[1].v = msg->twist.twist.linear.x;
    lcl[0].w = msg->twist.twist.angular.z;
	

    lcl[0].altering();
}


void 
Complement::imuCallback(const sensor_msgs::Imu::Ptr imu){

	if(!lcl[1].flag){	
		lcl[1].start();
    	lcl[1].flag = true;
    }

	lcl[1].w = imu->angular_velocity.z;
	lcl[1].w -= drift_dyaw;	
	// lcl[1].w = lcl[1].w * (-1);	
	lcl[1].altering();

	//回転
    if(lcl[1].w < -0.20 || 0.20 < lcl[1].w){
		cout<<"\r回転なう"<<flush;
    num = 10;
    // num = 0;
    }

    else {
		cout<<"\r-------"<<flush;
		num = 0;
	}
}


void 
Complement::ndtCallback(const nav_msgs::Odometry msg){
	
	if(!lcl[2].flag){	
		lcl[2].start();
        lcl[2].flag = true;
    }

    lcl[2].x = msg.pose.pose.position.x;
	lcl[2].y = msg.pose.pose.position.y;
	lcl[2].yaw = msg.pose.pose.orientation.z;
	
}


void 
Complement::ekfCallback(const nav_msgs::Odometry msg){

	if(!lcl[3].flag){	
		lcl[3].start();
		lcl[3].flag = true;
    }

    lcl[3].x = msg.pose.pose.position.x;
	lcl[3].y = msg.pose.pose.position.y;
	lcl[3].yaw = msg.pose.pose.orientation.z;
	
}

void 
Complement::initposeCallback(const geometry_msgs::PoseStampedConstPtr& msg){
	
    double qr,qp,qy;
    tf::Quaternion quat(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
    tf::Matrix3x3(quat).getRPY(qr, qp, qy);   

	lcl[type].x = msg->pose.position.x;
	lcl[type].y = msg->pose.position.y;
	lcl[type].yaw = qy;

	flag = true;

}


int 
Complement::start(){
	
// 0:odom 1:imu 2:ndt 3:ekf


	lcl_.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている
	lcl_.pose.pose.position.x = lcl[type].x;
	lcl_.pose.pose.position.y = lcl[type].y;
	lcl_.pose.pose.position.z = 0.0;
	lcl_.pose.pose.orientation.z = lcl[type].yaw;

	lcl_pub.publish(lcl_);

	lcl_vis.header.stamp = ros::Time::now(); //timestampのメッセージを送ろうとしている
	lcl_vis.pose.pose.position.x = lcl[type].x;
	lcl_vis.pose.pose.position.y = lcl[type].y;
	lcl_vis.pose.pose.position.z = z_height;	
	lcl_vis.pose.pose.orientation.z = sin(lcl[type].yaw*0.5);
	lcl_vis.pose.pose.orientation.w = cos(lcl[type].yaw*0.5);

	lcl_vis_pub.publish(lcl_vis);


	number.data = num;

	lcl_hantei_pub.publish(number);//0:直線 1:カーブ


	transform.setOrigin( tf::Vector3(lcl[type].x, lcl[type].y, z_height) );
	tf::Quaternion q;
	q.setRPY(0, 0, lcl[type].yaw);

	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, lcl_.header.stamp , HEADER_FRAME, CHILD_FRAME));

}


int main (int argc, char** argv){
	ros::init(argc,argv,"imu_complement");
	ros::NodeHandle n;
	ros::NodeHandle priv_nh("~");

	cout<<"------complement start---------"<<endl;
	Complement complement(n,priv_nh);
	complement.spin();

	return 0;
}
