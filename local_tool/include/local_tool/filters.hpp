#ifndef _FILTERS_HPP_
#define _FILTERS_HPP_

#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>


const static int N = 200;
const static int M = 200;

//filterは3×3を想定している(画像処理とかの平均化フィルタ)
const static int N_F = 3;
const static int M_F = 3;

void map_reader(float size, std::string file,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_map_cloud);

void voxel_grid(float size,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud);

//平滑化
void smoothing(float input_data[N][M],int filter[N_F][N_F],float output_data[N][M]);

//勾配を算出
void grad(float output[N][M],float out_x[N][M],float out_y[N][M]);

#endif

