#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <std_msgs/Float64.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/thread/thread.hpp>

#include "std_msgs/Int32.h"

#include <math.h>
#include <time.h>
#include <unistd.h>

using namespace std;

bool ndt_flag = false;
bool odo_flag = true;

bool start_flag = true;
// bool start_flag = false;


ros::Publisher pub;
sensor_msgs::PointCloud2 vis_before_laser;
sensor_msgs::PointCloud2 vis_after_laser;
nav_msgs::Odometry odo;
nav_msgs::Odometry odo_pub;

std_msgs::Int32 score_num;


float x_now,y_now;
float z_now = 0;
float yaw_now = 0;
// const float laser_limit = 2000;
const float laser_limit = 20;


pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr after_cloud (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZI>::Ptr before_cloud (new pcl::PointCloud<pcl::PointXYZI>);


float distance(float x,float y){
    float ans; 


    ans =  sqrt(x*x+y*y);

    return ans;

}




void odomCallback(nav_msgs::Odometry msg)//0.2進んだらtrueとかにしたい
{

    odo = msg;
    // if(odo_flag){
    x_now = odo.pose.pose.position.x;
    y_now = odo.pose.pose.position.y;
 
    yaw_now = odo.pose.pose.orientation.z;
    // }
    // odo_flag = false; 

}





void z_lcl_Callback(std_msgs::Float64 number){

    z_now = number.data + 0.7;

}

//
// void velo_clear_Callback(const sensor_msgs::PointCloud2ConstPtr& input)
// {
//
//     float d = 0 ;
//     ndt_flag = true;
//
//
//     // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
//     pcl::fromROSMsg (*input, *input_cloud);
//
//     //limit_laserを設定
// 	after_cloud->points.clear();
// 	
//     size_t velodyne_size = input_cloud->points.size();
// 	for(size_t i = 0; i < velodyne_size; i++){
// 		pcl::PointXYZI temp_point;
// 		temp_point.x = input_cloud->points[i].x; 
// 		temp_point.y = input_cloud->points[i].y;
// 		temp_point.z = input_cloud->points[i].z ;
//
//         temp_point.intensity = input_cloud->points[i].intensity;//ztointensity
//
//
//         d = distance(temp_point.x , temp_point.y);
//
//
//         if(3.5 < d && d <= laser_limit) after_cloud->points.push_back(temp_point);
//  
//     }
//
// }
//


void velo_Callback(const sensor_msgs::PointCloud2ConstPtr& input)
{

    float d = 0 ;
    ndt_flag = true;


    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::fromROSMsg (*input, *input_cloud);

    //limit_laserを設定
	after_cloud->points.clear();
	
    size_t velodyne_size = input_cloud->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZI temp_point;
		temp_point.x = input_cloud->points[i].x; 
		temp_point.y = input_cloud->points[i].y;
        temp_point.z = input_cloud->points[i].z ;
        
        temp_point.intensity = input_cloud->points[i].intensity;//ztointensity

        d = distance(temp_point.x , temp_point.y);
        
        // if( 2.5 < d  && d <= laser_limit) after_cloud->points.push_back(temp_point);
        if( !((fabs(temp_point.y) < 2) && ((temp_point.x >-5 ) && (temp_point.x < 0))) )after_cloud->points.push_back(temp_point);
        

    }

}


////////////////////////
double law_of_cosines(double a, double b ,double angle_c){//余弦定理
    double c;

    c = a*a + b*b - 2*a*b*cos(angle_c);
    
    return sqrt(c);

}

double reverse_cosines(double a, double b ,double c){//逆余弦定理
    double angle_c;

    angle_c = (c*c + b*b - a*a) / (2 * b * c);
    
    return acos(angle_c);

}


double fix_function(double x_befo, double y_befo, double x_velo, double y_velo){
    
    double angle_b,angle_c;
    double a,b,c;
   
    if(x_befo == 0){
        angle_c =  atan(y_velo/x_velo);//a to b angle
    }


    else{
        // if(fabs(x_velo) < 0.001){
        //     angle_c =  atan(y_befo/x_befo) + M_PI/2 ;//a to b angle
        // }
        //
        // else{
            angle_c =  atan(y_befo/x_befo) + atan(y_velo/x_velo);//a to b angle
        // }
    }

    // cout <<"えっくす" <<x_befo<<"わい"<< y_befo<<endl;
    // cout <<"ndtえっくす" <<x_velo<<"ndtわい"<< y_velo<<endl;
    // cout <<"あるふぁー" <<atan(y_befo/x_befo)<<"そうたい"<< atan(y_velo/x_velo)<<endl;
    // cout <<"がんまー" <<angle_c<<endl;


    //length
    a = sqrt(x_befo * x_befo + y_befo * y_befo);
    b = sqrt(x_velo * x_velo + y_velo * y_velo);

    //余弦定理
    c = law_of_cosines(a, b, angle_c);

    //逆余弦定理
    angle_b = reverse_cosines(a, b, c);

    // cout <<"べーたー" <<angle_b<<endl;
    return angle_b;
}
///////////////////////////




int main (int argc, char** argv)
{
    double l_roll, l_pitch, l_yaw;//角度etc
    
    float angle_b;

    float x_after, y_after;
    float length;

    ros::init(argc, argv, "slam_ndt2");
    
    ros::NodeHandle n;

    //loop_rateをあげるべき
    ros::Rate loop_rate(2);
    // ros::Rate loop_rate(5);
    // ros::Rate loop_rate(10);
    

    cout <<"---shape start----"<< endl;

    ros::Publisher ndt_pub = n.advertise<nav_msgs::Odometry>("/sq_ndt_data", 1000);
    ros::Publisher vis_before_laser_pub = n.advertise<sensor_msgs::PointCloud2>("/before_ndt", 1000);//ndtかけた点をみる
    ros::Publisher vis_after_laser_pub = n.advertise<sensor_msgs::PointCloud2>("/after_ndt", 1000);//ndtかけた点をみる


    ros::Publisher score_pub = n.advertise<std_msgs::Int32>("/fitness_score", 1000);

    
    ros::Subscriber velo_z_sub = n.subscribe("/velodyne_obstacles", 1000, velo_Callback); //////形状情報+受光強度
    ros::Subscriber odom_sub = n.subscribe("/lcl_ekf", 1000, odomCallback);//lclから現在地求む

    tf::TransformBroadcaster br;
    tf::Transform transform;

    odo.pose.pose.position.x = 0.0;
    odo.pose.pose.position.y = 0.0;


    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;//default value
    
    //////x,y,z,roll,pitch,yaw]の各々のメートル・ラジアンの 最小限・許容・増加微分 
    
    //Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.001);//[x,y,z,roll,pitch,yaw]の各々のメートル・ラジアンの 最小限・許容・増加微分 
    //増加微分が閾値を下回ると位置合わせが終了
    
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.01);
    //最大限の値を下回ったら
    
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (1.0);
    
    // Setting max number of registration iterations.最高の試行回数を設定
    ndt.setMaximumIterations (35);

    //int co = 0;
	while(ros::ok()){

        if(ndt_flag && odo_flag && start_flag){

            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_laser_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
            
            approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05); 
            approximate_voxel_filter.setInputCloud (after_cloud);
            approximate_voxel_filter.filter (*filtered_laser_cloud); 
            
            before_cloud = filtered_laser_cloud;

            start_flag = false;
        }    
        if(ndt_flag && odo_flag && !start_flag){
        // Loading first scan of room.
        
        //tutorialのinputとか  
        
            // Filtering input scan to roughly 10% of original size to increase speed of registration.
            
            // voxel化
            pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_laser_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            pcl::ApproximateVoxelGrid<pcl::PointXYZI> approximate_voxel_filter;
            
            approximate_voxel_filter.setLeafSize (0.05, 0.05, 0.05);
            approximate_voxel_filter.setInputCloud (after_cloud);
            approximate_voxel_filter.filter (*filtered_laser_cloud);
            // std::cout << "1スキャン前 " << before_cloud->size ()<< std::endl;
            // std::cout << "リアルスキャン " << filtered_laser_cloud->size ()<< std::endl;
           



            // Setting point cloud to be aligned.点群を整列させるようにセット
            // ndt.setInputSource (filtered_laser_cloud);//input lazerによる点群
            ndt.setInputSource (before_cloud);//input lazerによる点群
            
            // Setting point cloud to be aligned to.
            

            ndt.setInputTarget (filtered_laser_cloud);//target 現在のスキャン
            
            //////// Set initial alignment estimate found using robot odometry.

            Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
            Eigen::Translation3f init_translation (0, 0, 0); //昔の位置にすべき

            Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
           
              

            // Calculating required rigid transform to align the input cloud to the target cloud.
            pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
            ndt.align (*output_cloud, init_guess);
            



            Eigen::Matrix4f a;
            a = ndt.getFinalTransformation ();////4×4の行列         
                                              //最適の角度・長さを検出
 
            tf::Matrix3x3 mat_l;
			mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
						   static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
						   static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));
			
			mat_l.getRPY(l_roll, l_pitch, l_yaw, 1);

 //////////////////変換行列から得た答え           
/*
			cout<<"t(0,3) : "<<a(0, 3)<<endl;
			cout<<"t(1,3) : "<<a(1, 3)<<endl;
			cout<<"l_roll : "<<l_roll<<endl;
			cout<<"l_pitch : "<<l_pitch<<endl;
            cout<<"l_yaw : "<<l_yaw<<endl;
*/

//            cout<<"l_yaw : "<<l_yaw<<endl;
////////////////////////before////////////////////////////////////////
/*
            angle_b = fix_function(x_now, y_now, a(0, 3), a(1, 3));

            x_after = cos(angle_b) * a(0, 3) - sin(angle_b) * a(1, 3);
            y_after = sin(angle_b) * a(0, 3) + cos(angle_b) * a(1, 3);

            odo_pub.pose.pose.position.x =  x_after; 
            odo_pub.pose.pose.position.y =  y_after; 
            odo_pub.pose.pose.orientation.z = l_yaw;  
*/
/////////////////////////after//////////////////////

            // cout <<"score"<<ndt.getFitnessScore()<<endl;

            if(ndt.getFitnessScore() < 0.3) score_num.data = 0; 
            // if(ndt.getFitnessScore() < 0.001) score_num.data = 0; 
            else score_num.data = 10; 



            double xx,yy;
            
            angle_b = fix_function(x_now, y_now, -a(0, 3), -a(1, 3));
            // angle_b = fix_function(x_now, y_now, x_after , y_after);
            // xx = cos(angle_b) * x_now - sin(angle_b) * y_now;
            // yy = sin(angle_b) * x_now + cos(angle_b) * y_now;

            // xx = -cos(angle_b) * a(0, 3) + sin(angle_b) * a(1, 3);
            // yy = -sin(angle_b) * a(0, 3) - cos(angle_b) * a(1, 3);

            xx =  -a(0, 3);
            yy =  -a(1, 3);
            
            length = sqrt(xx*xx + yy*yy);

            // cout<<"yaw"<<  yaw_now - l_yaw<< endl; 
            x_after = length * cos(yaw_now-l_yaw);
            y_after = length * sin(yaw_now-l_yaw);

            
            odo_pub.pose.pose.position.x =  x_now + x_after; 
            odo_pub.pose.pose.position.y =  y_now + y_after; 
            odo_pub.pose.pose.orientation.z = yaw_now - l_yaw;  

            // cout<<"a(0, 3)"<< a(0, 3) <<"a(1, 3)"<< a(1, 3) << endl; 
            // cout<<"x方向"<< x_after <<"y方向"<< y_after << endl; 
            // cout<<"sinx方向"<< xx <<"yy方向"<< yy << endl; 
            
            ///////////////////laserからmap座標への変換式


            ndt_pub.publish(odo_pub);               

            //scanをpub
            
            pcl::toROSMsg(*before_cloud , vis_before_laser); //tの結果           
            pcl::toROSMsg(*filtered_laser_cloud , vis_after_laser);           

            vis_before_laser.header.frame_id = "/velodyne"; //laserのframe_id
            vis_before_laser.header.stamp = ros::Time::now(); //laserのframe_id


            vis_after_laser.header.frame_id= "/velodyne"; //laserのframe_id
            vis_after_laser.header.stamp = ros::Time::now(); //laserのframe_id

            vis_before_laser_pub.publish(vis_before_laser);
            vis_after_laser_pub.publish(vis_after_laser);

            before_cloud = filtered_laser_cloud;

            score_pub.publish(score_num);


            odo_flag = false;

            }

        // usleep(50000);
        odo_flag =true;

        ros::spinOnce();
        loop_rate.sleep();
    } 
    
    return (0);
}


