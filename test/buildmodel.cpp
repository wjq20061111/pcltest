#include "filter.h"
#include "VFH.h"
#include "viewer.h"
#include "segement.h"
#include "config.h"
#include "filter.h"
#include <iostream>
#include <sstream>

int main (int argc, char** argv)
{
	
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	for(int fi=0;fi<14;fi++)
	{
		pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr filter_cloud (new pcl::PointCloud<PointT>);
		std::stringstream sso;
		sso<< "cap/orig" << fi<<".pcd";
		std::stringstream ssf;
		ssf<< "model/cup" << fi<<".pcd";
		std::stringstream ssv;
		ssv<< "model/cup" << fi<<"_vfh.pcd";

		reader.read (sso.str(), *src_cloud);
		passthroughfilter(src_cloud,src_cloud,'z',0,1);
			//pclviewer(src_cloud);
		pcl::PointCloud<PointT>::Ptr target (new pcl::PointCloud<PointT>);


		pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
		pcl::SACSegmentation<PointT> seg;
  	// Optional
		seg.setOptimizeCoefficients (true);
  	// Mandatory
		seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType (pcl::SAC_RANSAC);
		seg.setDistanceThreshold (0.01);
		seg.setInputCloud (src_cloud);
		seg.segment (*plane_inliers, *plane_coefficients);
		pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
		extractinliers(src_cloud,plane_inliers,cloud_plane);	
		Eigen::Vector4f centroid;
		if((std::fabs(plane_coefficients->values[1])>0.8)&&
			(std::fabs(plane_coefficients->values[0])<0.2)&&
			(std::fabs(plane_coefficients->values[2])<0.2))
		{
			pcl::compute3DCentroid (*cloud_plane, centroid);
			
		}
		pcl::PointCloud<PointT>::Ptr cloud_trueplane (new pcl::PointCloud<PointT>);		
		passthroughfilter(src_cloud,cloud_trueplane,'y',centroid[1]-0.2,centroid[1]+0.2);
		
		std::vector<pcl::PointIndices> cluster_indices;
		



	extractinliers(src_cloud,plane_inliers,cloud_plane,true);

threedfilter(cloud_plane,cloud_plane);
	passthroughfilter(cloud_plane,cloud_plane,'z',0,3);
	//pclviewer(cloud_plane);

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_plane);

		
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (80);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_plane);
	ec.extract (cluster_indices);

		
		extractinliers(cloud_plane,
			boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices( cluster_indices[1] ) ),
			target);
		pclviewer(target);
		//downsamplefilter(src_cloud,filter_cloud);

		// writer.write<PointT>(ssf.str(),*target,false);
		// pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
		// calVFH(target,vfhs);
		// writer.write<pcl::VFHSignature308>(ssv.str(),*vfhs,false);
		
	}

	return 0;
}