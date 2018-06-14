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

class Mesh
{
	private:
		ros::NodeHandle n;
		ros::Rate r;
		ros::Publisher input_pub;
		ros::Publisher output_pub;

		double MU;
		double M_NEIGHBORS;//max
		double G_RADIUS;
		double K_SEARCH;

		string file_input;
		
		bool NORMAL_C;

		pcl::PolygonMesh::Ptr triangles;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ;

	public:
		Mesh(ros::NodeHandle& n);

		void create_polygon(void);
		void vis_polygon(void);

};

Mesh::Mesh(ros::NodeHandle &n) :
	r(10)
{

	n.getParam("mesh_mu",MU);
	n.getParam("mesh_max_neighbors",M_NEIGHBORS);
	n.getParam("mesh_radius",G_RADIUS);
	n.getParam("mesh_normalconsistency",NORMAL_C);
	n.getParam("normal_ksearch",K_SEARCH);
	n.getParam("input/cloud",file_input);

	input_pub = n.advertise<sensor_msgs::PointCloud2>("/input_cloud", 10);
	output_pub = n.advertise<sensor_msgs::PointCloud2>("/output_cloud", 10);

	triangles.reset (new pcl::PolygonMesh());
	cloud.reset (new pcl::PointCloud<pcl::PointXYZ>);

}





void
Mesh::create_polygon(void){

	pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZI>);
	// Load input file into a PointCloud<T> with an appropriate type
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
	nor.setKSearch (K_SEARCH);
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

	// Set the maximum distance between connected points (maximum edge length)
	// gp3.setSearchRadius (0.025);
	gp3.setSearchRadius (G_RADIUS);

	// Set typical values for the parameters
	gp3.setMu (MU);//defalut 2.5
	gp3.setMaximumNearestNeighbors (M_NEIGHBORS);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
	gp3.setMinimumAngle(M_PI/18); // 10 degrees
	gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	gp3.setNormalConsistency(NORMAL_C);
	// gp3.setNormalConsistency(true);

	// Get result
	gp3.setInputCloud (cloud_with_normals);
	gp3.setSearchMethod (tree2);
	gp3.reconstruct (*triangles);

	// Additional vertex information
	std::vector<int> parts = gp3.getPartIDs();
	std::vector<int> states = gp3.getPointStates();

	cout <<triangles->polygons.size() <<"triangles created" << endl;


	//ros
	// pcl::fromPCLPointCloud2 (triangles->cloud, *output_cloud);
	// sensor_msgs::PointCloud2 pc, pc2;
	// pcl_msgs::PolygonMesh pc3;
	// pcl::toROSMsg(*cloud, pc);
	// pcl::toROSMsg(*output_cloud, pc2);
	// pc3.cloud = pc2;


}

void
Mesh::vis_polygon(void)
{
	// 点群のビューア
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");

	viewer.setBackgroundColor (0.0, 0.2, 0.6);

	viewer.addPolygonMesh(*triangles);

	while (!viewer.wasStopped ())
	{
		viewer.spinOnce(10);

		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}


}



int main(int argc, char** argv){
	ros::init(argc, argv, "mesh");
	ros::NodeHandle n;

	cout<<"-------mesh ok--------"<<endl;

	Mesh mesh(n);

	mesh.create_polygon();
	mesh.vis_polygon();

	ros::spin();

	return 0;
}

