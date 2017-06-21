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
	reg.setMinClusterSize (100);
	reg.setMaxClusterSize (1000000);
	reg.setSearchMethod (tree);
	reg.setNumberOfNeighbours (30);
	reg.setInputCloud (cloud);
  	//reg.setIndices (indices);
	reg.setInputNormals (normals);
	reg.setSmoothnessThreshold (5.0 / 180.0 * M_PI);
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

void calEuclideanClusterExtraction(const pcl::PointCloud<PointT>::Ptr  &cloud , 
	std::vector <pcl::PointIndices> &cluster_indices
	)
{
	pcl::PointCloud<PointT>::Ptr copy_src_cloud (new pcl::PointCloud<PointT>());
	pcl::copyPointCloud(*cloud, *copy_src_cloud); 
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr temp_cloud_plane (new pcl::PointCloud<PointT> ());
	pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>());
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.01);

	int i=0, nr_points = (int) copy_src_cloud->points.size ();
	while (copy_src_cloud->points.size () > 0.5 * nr_points)
	{
    	// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (copy_src_cloud);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

    	// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (copy_src_cloud);
		extract.setIndices (inliers);
		extract.setNegative (false);

    	// Get the points associated with the planar surface
		extract.filter (*temp_cloud_plane);
//pclviewer(temp_cloud_plane);
		std::cout << "PointCloud representing the planar component: " << temp_cloud_plane->points.size () << " data points." << std::endl;

    	// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*copy_src_cloud = *cloud_f;
	}
		passthroughfilter(copy_src_cloud,copy_src_cloud,'z',0,3);
//pclviewer(copy_src_cloud);

  	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (copy_src_cloud);

	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (80);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (copy_src_cloud);
	ec.extract (cluster_indices);
}