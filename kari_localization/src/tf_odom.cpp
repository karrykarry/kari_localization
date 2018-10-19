//
//tfを取ったbagを再生し、matching_base_linkやLIDARのtfを再生するため
//
// /matching_base_link_clone がmoveする
// 関係はmap→matching_base_link を map→  matching_base_lnk_clone と同じにするもの
// 
//

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

using namespace std;


class Broad
{
	private:
		ros::NodeHandle n;
		ros::Rate r;
		ros::Subscriber odom_sub;

		bool flag;
		string PARENT_FRAME;
		string CHILD_FRAME;
		string ODOM_SUB;

		nav_msgs::Odometry odo;
		tf::TransformBroadcaster br;
		tf::Transform transform;

	public:
		Broad(ros::NodeHandle &n);
		void odomCallback(const nav_msgs::Odometry msg);
		void prepare();

		bool spin()
		{
			ros::Rate loop_rate(r);

			while(ros::ok()){
				if(flag) prepare();
				ros::spinOnce();
				loop_rate.sleep();
			}
			return true;
		}

};

Broad::Broad(ros::NodeHandle &n) :
	r(10),
	flag(true)
{
	n.getParam("parent_frame",PARENT_FRAME);
	n.getParam("child_frame",CHILD_FRAME);
	n.getParam("odom_sub",ODOM_SUB);

	odom_sub = n.subscribe(ODOM_SUB, 100, &Broad::odomCallback, this);
}


void 
Broad::odomCallback(const nav_msgs::Odometry msg){
    
    odo = msg;
    
    transform.setOrigin( tf::Vector3(odo.pose.pose.position.x, odo.pose.pose.position.y, 0.0) );
    tf::Quaternion q;
    q.setRPY(odo.pose.pose.orientation.x, odo.pose.pose.orientation.y, odo.pose.pose.orientation.z);
    transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), PARENT_FRAME, CHILD_FRAME));

}

void
Broad::prepare()
{

	transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
	transform.setRotation( tf::Quaternion(0, 0, 0) );
	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), PARENT_FRAME, CHILD_FRAME));
	flag = false;

}




int main(int argc, char** argv){
	ros::init(argc, argv, "tf_odom");
	ros::NodeHandle n;

	cout<<"------tf start-----"<< endl;

	Broad broad(n);

	broad.spin();

	return 0;
}
