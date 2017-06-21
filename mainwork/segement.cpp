#include "segement.h"

void findplane(const pcl::PointCloud<PointT>::Ptr  &cloud ,
	pcl::PointIndices::Ptr &inliers, 
	pcl::ModelCoefficients::Ptr &coefficients)
{
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
  	// Optional
	seg.setOptimizeCoefficients (true);
  	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setAxis(Eigen::Vector3f (0.0, 1.0, 0.0));
	seg.setEpsAngle(3.14/180*10); 
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);
}

void ECExtraction(const pcl::PointCloud<PointT>::Ptr  &cloud_filtered , 
	std::vector<pcl::PointIndices> &cluster_indices)
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_filtered);

	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);
}

void RegionGrowingSeg(const pcl::PointCloud<PointT>::Ptr  &cloud , 
	std::vector <pcl::PointIndices> &clusters,
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr &colored_cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	normalcal(cloud,normals);
	pcl::RegionGrowing<PointT, pcl::Normal> reg;
	reg.setMinClusterSize (50);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud);
  	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold (1.0);
	reg.extract (clusters);
	colored_cloud = reg.getColoredCloud ();
}

void RegionGrowingRGBSeg(const pcl::PointCloud<PointT>::Ptr  &cloud , 
	std::vector <pcl::PointIndices> &clusters,
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr &colored_cloud)
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	pcl::RegionGrowingRGB<PointT> reg;
	reg.setInputCloud (cloud);
	//reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (10);
	reg.setPointColorThreshold (6);
	reg.setRegionColorThreshold (5);
	reg.setMinClusterSize (600);
	reg.extract (clusters);
	colored_cloud = reg.getColoredCloud ();
}