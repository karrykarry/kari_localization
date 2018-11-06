#include "pc2transform4map_main.hpp"

/*自分付近の最下点を算出
 *
 */
#define MIN(x,y) ((x) < (y) ? (x) : (y))
#define MAX(x,y) ((x) > (y) ? (x) : (y))


pcl::PointCloud<pcl::PointXYZ>::Ptr local_map_cloud (new pcl::PointCloud<pcl::PointXYZ>);

4Map_main::4Map_main()
{
	grid_dim_ = 8;
	m_per_cell_ = 0.5;
	map_limit = 4.0;
}


void local_map(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point)
{
    //limit_laserを設定
	local_map_cloud->points.clear();
	
    size_t velodyne_size = input_point->points.size();
	for(size_t i = 0; i < velodyne_size; i++){
		pcl::PointXYZ temp_point;
		temp_point.x = input_point->points[i].x; 
		temp_point.y = input_point->points[i].y;
		temp_point.z = input_point->points[i].z;

         
        //if(d <= map_limit && (-0.5 < temp_point.z && temp_point.z <5.0)){
		if((map_limit * (-1) + x_now <= temp_point.x && temp_point.x  <= map_limit + x_now) && (map_limit *(-1) + y_now <= temp_point.y && temp_point.y <= map_limit + y_now)) {//つくば
	
			local_map_cloud->points.push_back(temp_point);
		}
	}
}


float get_min(pcl::PointCloud<pcl::PointXYZ>::Ptr input_point){
       
    float min[grid_dim_][grid_dim_];
    bool init[grid_dim_][grid_dim_];
	
	fill(min[0],min[grid_dim_],100);
	fill(init[0],init[grid_dim_],0);

	float ans;
	int count;
	count =0;
	ans =0;

    size_t velodyne_size = input_point->points.size();
    //各グリッドの最下点を算出
	for (size_t i = 0; i < velodyne_size; ++i) {
        pcl::PointXYZ scan;
        
        scan.x = input_point->points[i].x - x_now;
        scan.y = input_point->points[i].y - y_now;
        scan.z = input_point->points[i].z;

        int x = ((grid_dim_/2)+scan.x /m_per_cell_);
        int y = ((grid_dim_/2)+scan.y /m_per_cell_);

        if (x >= 0 && x < grid_dim_  && y >= 0 && y < grid_dim_) {
			if (!init[x][y]) {
                min[x][y] = scan.z;
                init[x][y] = true;
            } 
			else min[x][y] = MIN(min[x][y], scan.z);
        }
    }

	// tkb用
	//minimumの近傍を平均する
	//
	for (int i=0;i<grid_dim_;i++){
		for (int j=0;j<grid_dim_;j++){
			if(init[i][j]){
				ans = (ans * count + min[i][j])/(count+1);
				count++;
			}
		}
	}
	
	return ans;
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "height_min2");
  	ros::NodeHandle n;
	ros::Rate roop(10);

	ros::Publisher pub_height  = n.advertise<std_msgs::Float64>("/height", 10);	
  
    ros::Subscriber odom_sub = n.subscribe("/lcl_ekf", 1000, lcl_Callback);//lclから現在地求む

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_IN(new pcl::PointCloud<pcl::PointXYZ>);
    
	string map_file;
	n.getParam("map_file",map_file);
	map_reader(0.2,map_file,cloud_IN);
   
	std_msgs::Float64 num;
	
	while(ros::ok()){

		local_map(cloud_IN);	
        num.data = get_min(local_map_cloud);

		pub_height.publish(num);

		roop.sleep();
		ros::spinOnce();
	}
	return (0);
}

