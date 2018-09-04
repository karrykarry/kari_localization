#include "GICP_probability.h"

GICP::GICP(void){
	MAX_CORRESPONDENCE_DIST = 0.3f;	// 最近傍点探索時、srcに対するtgtの最近傍点が、何mまでなら採用するかの閾値
	PROB_THRESHOLD = 0.2f;			// Subscribeした点群の確率何%以上を採用するかの閾値
};

void GICP::setMaxCorrespondenceDist(float mcd) { this->MAX_CORRESPONDENCE_DIST = mcd; };
void GICP::setProbThreshold(float pt) { this->PROB_THRESHOLD = pt; };

Eigen::Affine3f
GICP::getTF(geometry_msgs::Pose pose){
	Eigen::Quaternionf quat = Eigen::Quaternionf(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
	Eigen::Translation<float,3> trans = Eigen::Translation<float,3>(pose.position.x, pose.position.y, pose.position.z);
	Eigen::Affine3f affine = trans * quat;
	
	return affine;
};

void
GICP::search_correspondingPC(pcl::PointCloud<pcl::PointXYZINormal>::Ptr rawTgt, pcl::PointCloud<pcl::PointXYZINormal>::Ptr rawSrc, pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformSrc, pcl::PointCloud<pcl::PointXYZINormal>::Ptr distSrc, pcl::PointCloud<pcl::PointXYZINormal>::Ptr transformTgt, pcl::PointCloud<pcl::PointXYZINormal>::Ptr distTgt, Eigen::Affine3f svd){
	pcl::PointCloud<pcl::PointXYZINormal>::Ptr src_transform (new pcl::PointCloud<pcl::PointXYZINormal>);
	pcl::search::KdTree<pcl::PointXYZINormal> kdtree (new pcl::search::KdTree<pcl::PointXYZINormal>);
	transformTgt->points.clear();		transformTgt->points.shrink_to_fit();
	distTgt->points.clear();			distTgt->points.shrink_to_fit();
	transformSrc->points.clear();		transformSrc->points.shrink_to_fit();
	distSrc->points.clear();			distSrc->points.shrink_to_fit();
	std::vector<int> idx(1);
	std::vector<float> dist(1);
	int reject_cnt =0;

	//cout << "	input cloud size : " << last_cloud_raw->points.size() << endl;
	/*---- srcを変換行列を使って変換するよ！----*/
	pcl::transformPointCloud(*rawSrc, *src_transform, svd);
	/*---- 変換後のsrc(cloud_transform)に対応するtgtを探索するよ！----*/
	kdtree.setInputCloud(rawTgt);
	size_t cloud_size = src_transform->points.size();
	for(size_t i=0; i<cloud_size; i++){
		kdtree.nearestKSearch(src_transform->points[i], 1, idx, dist);
		if(dist[0] <= MAX_CORRESPONDENCE_DIST){
			distSrc->points.push_back(rawSrc->points[i]);
			distTgt->points.push_back(rawTgt->points[idx[0]]);
			if(src_transform->points[i].intensity >= PROB_THRESHOLD){
				transformSrc->points.push_back(rawSrc->points[i]);
				transformTgt->points.push_back(rawTgt->points[idx[0]]);
			}
		}else{
			reject_cnt++;
		}
	}
};

Eigen::Affine3f
GICP::calc_RT(pcl::PointCloud<pcl::PointXYZINormal>::Ptr src, pcl::PointCloud<pcl::PointXYZINormal>::Ptr tgt){
	boost::shared_ptr<pcl::registration::TransformationEstimationSVD<pcl::PointXYZINormal, pcl::PointXYZINormal> > estPtr;
	//boost::shared_ptr<pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZINormal, pcl::PointXYZINormal> > estPtr;
	Eigen::Affine3f transformation_est;
	estPtr.reset (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	//estPtr.reset (new pcl::registration::TransformationEstimationPointToPlane<pcl::PointXYZINormal, pcl::PointXYZINormal>());
	estPtr->estimateRigidTransformation(*src, *tgt, transformation_est.matrix());

	return transformation_est;
};
