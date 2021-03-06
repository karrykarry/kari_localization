#include "mathematics.hpp"

#include <tf/transform_broadcaster.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>



void calc_rpy(Eigen::Matrix4f a,double &roll,double &pitch,double &yaw){
	tf::Matrix3x3 mat_l;
	mat_l.setValue(static_cast<double>(a(0, 0)), static_cast<double>(a(0, 1)), static_cast<double>(a(0, 2)),
			static_cast<double>(a(1, 0)), static_cast<double>(a(1, 1)), static_cast<double>(a(1, 2)),
			static_cast<double>(a(2, 0)), static_cast<double>(a(2, 1)), static_cast<double>(a(2, 2)));

	mat_l.getRPY(roll, pitch, yaw, 1);
}

float distance(float x,float y){
    float ans; 

    ans =  sqrt(x*x+y*y);

    return ans;

}

void xy2rtheta(double x,double y, double &r, double &theta){
	
	r = sqrt(x*x +y*y);

	theta = std::atan2(y,x) + 2*M_PI;

	if(theta > 2*M_PI) theta -= 2*M_PI;
	theta = theta/M_PI*180;

}

