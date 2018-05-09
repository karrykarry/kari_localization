#ifndef __IMU_TEST_H__
#include <time_util/stopwatch.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;

class Localization{
public:
	double x,y,pitch,roll,yaw,v,w;
	double p_,r_;
	double d_yaw,d_pitch,d_roll;
    double z;
    double time;

    double dt;

	double t;

	Stopwatch sw;
	
	Localization();
	~Localization();
	void showState();
	void gettime();
	void altering();
	void altering2();
	void altering3();
	void altering4();
	void alter_dyaw();
	void start();
};

#include "impl/imu_test.hpp"
#endif
