/*/waypointの途中から自己位置とwp_nowをpubする
 * 
 * author : R.Kusakari
 *
 */
#include <ros/ros.h>
#include <string>
#include <vector>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <fstream>

using namespace std;

nav_msgs::Odometry wp_pub;

double YawCalculater(double x1,double x2,double y1,double y2)
{
	return atan2(y2-y1, x2-x1);
}


void inputFromTxt(const string& file_name,int wp_num)
{
    ifstream ifs(file_name.c_str());
    string buf;
    int wp_count=0;
    double x,y,yaw;
    geometry_msgs::Point pt; 
    if( !ifs ){
        ROS_FATAL( "Error:Non file!!!!!---------Is this file's name correct?\n------->%s\n",file_name.c_str());
        exit(0);
    }else{
        while(ifs && getline(ifs, buf)) {
            istringstream is( buf );
            is>>x>>y>>yaw; //代入
  			if(wp_num==wp_count){
				wp_pub.pose.pose.position.x = x;
				wp_pub.pose.pose.position.y = y;
			}

  			if((wp_num+1)==wp_count){
				wp_pub.pose.pose.orientation.z = 
				 	YawCalculater(wp_pub.pose.pose.position.x,x,wp_pub.pose.pose.position.y,y);
			}
            wp_count++;
        }
    }   
}

int input_num(){
	int output_num;
	cout<<"初期位置にするwp_num: " << flush;
	for ( cin >> output_num ; !cin ; cin >> output_num){
		cin.clear();
		cin.ignore();
		cout << "入力が間違っています。" << endl;
		cout<<"初期位置にするwp_num: " << flush;
	}
	return output_num;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "height_min2");
  	ros::NodeHandle n;
	ros::Rate loop(10);

	// ros::Publisher wp_now  = n.advertise<nav_msgs::Odometry>("/lcl_ekf", 10);	
	ros::Publisher wp_now  = n.advertise<nav_msgs::Odometry>("/initinal/pose", 10);	
   
	string wp_dir_path;
	string wp_txt;
	int wp_num;

	n.getParam("wp_dir_path", wp_dir_path);
	n.getParam("wp_txt", wp_txt);

	string waypoint_file_name = wp_dir_path + wp_txt;
	wp_num = input_num();	
	inputFromTxt(waypoint_file_name,wp_num);
	wp_pub.header.stamp = ros::Time::now();
	wp_pub.header.frame_id = "/map";
	
	wp_now.publish(wp_pub);
	
	sleep(0.5);
	return (0);
}
