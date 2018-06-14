//meshビューア

#include <ros/ros.h>
#include <stdio.h>
#include <iostream>
#include <string>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>
#include <pcl/PolygonMesh.h>

#include <boost/thread/thread.hpp>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_broadcaster.h>


#include <local_tool/filters.hpp>
#include <local_tool/mathematics.hpp>
#include <local_tool/registration.hpp>


using namespace std;


int main (int argc, char** argv)
{

	// double l_roll, l_pitch, l_yaw;//角度etc

	ros::init(argc, argv, "mesh");
	ros::NodeHandle n;
	ros::Rate roop(1);


	ros::Publisher ipt_pub = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 1);
	ros::Publisher out_pub = n.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1);
	
	ros::Publisher mesh_pub = n.advertise<pcl_msgs::PolygonMesh>("/mesh_cloud", 1);

	// Loading first scan of room.
	pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);

	tf::TransformBroadcaster br;
	tf::Transform transform;

	string file_input;

	n.getParam("input/cloud",file_input);


	// Load input file into a PointCloud<T> with an appropriate type
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PCLPointCloud2 cloud_blob;
	pcl::io::loadPCDFile(file_input, cloud_blob);
	pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
	//* the data should be available in cloud

	// Normal estimation*
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> nor;
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);
	nor.setInputCloud (cloud);
	nor.setSearchMethod (tree);
	nor.setKSearch (20);
	nor.compute (*normals);
	//* normals should not contain the point normals + surface curvatures

	// Concatenate the XYZ and normal fields*
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
	//* cloud_with_normals = cloud + normals

	// Create search tree*
	pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
	tree2->setInputCloud (cloud_with_normals);

	// Initialize objects
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
	// pcl::PolygonMesh triangles;
	pcl::PolygonMesh::Ptr triangles(new pcl::PolygonMesh());

	// Set the maximum distance between connected points (maximum edge length)
	// gp3.setSearchRadius (0.025);
	gp3.setSearchRadius (0.5);

	// Set typical values for the parameters
	gp3.setMu (10);//defalut 2.5
	gp3.setMaximumNearestNeighbors (100);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	// gp3.setNormalConsistency(false);
	gp3.setNormalConsistency(true);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	// pcl::io::saveVTKFile("/home/amsl/Desktop/mesh.vtk",triangles);

	cout <<triangles->polygons.size() <<"triangles created" << endl;

	pcl::fromPCLPointCloud2 (triangles->cloud, *output_cloud);
	// pcl_conversions::fromPCL (triangles->cloud, *output_cloud);

	
	sensor_msgs::PointCloud2 pc, pc2;
	pcl_msgs::PolygonMesh pc3;
	pcl::toROSMsg(*cloud, pc);
	pcl::toROSMsg(*output_cloud, pc2);

	pc3.cloud = pc2;


	// 点群のビューア
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	viewer.setBackgroundColor (0.0, 0.2, 0.6);
	// viewer.setBackgroundColor (0.0, 0.0, 0.0);

	// 原点に座標軸を追加
	// viewer.addCoordinateSystem(100.0, 0.0, 0.0, 0.0);

	// カメラ座標のセット
	// double camPos[] = { 0.0, 60.0, 350.0 };
	// double camLookAt[] = { 0.0, 60.0, 0.0 };
	// double camUp[] = { 0.0, 1.0, 0.0 };
	// viewer.setCameraPose(camPos[0], camPos[1], camPos[2], 
	// 		camLookAt[0], camLookAt[1], camLookAt[2], camUp[0], camUp[1], camUp[2]);

	// ビューアに点群を追加
	//  viewer.addPointCloud(filtered_cloud);
	// viewer.addPointCloud(cloud);

	// ビューアにポリゴンメッシュを追加
	 viewer.addPolygonMesh(*triangles);

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce(10);

		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


	// boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	// viewer->setBackgroundColor (0, 0, 0);
	// viewer->addPolygonMesh(*triangles,"meshes",0);


	// while(ros::ok()){
	// 	
    //
	// 	pc.header.frame_id  = "map";
	// 	pc2.header.frame_id = "map";
	// 	pc3.header.frame_id = "map";
    //
	// 	pc.header.stamp  = ros::Time::now();
	// 	pc2.header.stamp = ros::Time::now();
	// 	pc3.header.stamp = ros::Time::now();
    //
	// 	ipt_pub.publish(pc);
	// 	out_pub.publish(pc2);
	// 	
	// 	// mesh_pub.publish(pc3);
    //
	// 	roop.sleep();
	// 	ros::spinOnce();
	// }

	return (0);
}



