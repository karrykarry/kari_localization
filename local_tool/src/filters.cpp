#include "filters.hpp"

// void map_reader(std::string file,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_map_cloud){
void map_reader(float size, std::string file, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_map_cloud){

	pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile(file, *target_map_cloud);

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (size, size, size);
    approximate_voxel_filter.setInputCloud (target_map_cloud);
    approximate_voxel_filter.filter (*filtered_map_cloud);

}


void voxel_grid(float size,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud){

    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    
	approximate_voxel_filter.setLeafSize (size, size, size);
    approximate_voxel_filter.setInputCloud (cloud);
    approximate_voxel_filter.filter (*filtered_cloud);

}

void smoothing(float input_data[N][M],int filter[N_F][N_F],float output_data[N][M]){

	//周りを0にするために
	float swap[N+2][M+2] = {0};//N+2,M+2
	float swap_[N+2][M+2] = {0};//N+2,M+2

	//swapに保存
	for(int i=1;i< N+1;i++){
		for(int j=1;j< M+1;j++){
			swap[i][j] = input_data[i-1][j-1] ;
		}
	}

	for(int i=1;i< N+1;i++){
		for(int j=1;j< M+1;j++){
			swap_[i][j] = swap[i-1][j-1] * filter[0][0] + swap[i-1][j] * filter[0][1] + swap[i-1][j+1] * filter[0][2] ;
			swap_[i][j] += swap[i][j-1] * filter[1][0] + swap[i][j] * filter[1][1] + swap[i][j+1] * filter[0][2];
			swap_[i][j] += swap[i+1][j-1] * filter[2][0] + swap[i+1][j] * filter[2][1] + swap[i+1][j+1] * filter[2][2] ;
		
			output_data[i-1][j-1] = swap_[i][j];	
		}
	}

}


void grad(float output[N][M],float out_x[N][M],float out_y[N][M]){

	for(int i=0;i<N;i++){
		for(int j=0;j<M;j++){
			output[i][j] = sqrt(out_x[i][j] * out_x[i][j] + out_y[i][j] * out_y[i][j]);
		}
	}
}

