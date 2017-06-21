#include "VFH.h"

void calVFH(pcl::PointCloud<PointT>::Ptr &cloud,
	pcl::PointCloud<pcl::VFHSignature308>::Ptr &vfhs)
{
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
	normalcal(cloud,normals);

	// Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<PointT, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (normals);
	// alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

	// Create an empty kdtree representation, and pass it to the FPFH estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
	vfh.setSearchMethod (tree);

	// Compute the features
	vfh.compute (*vfhs);
	// vfhs->points.size () should be of size 1*
}