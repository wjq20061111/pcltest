#include "filter.h"

void threedfilter(const pcl::PointCloud<PointT>::Ptr  &cloud , 
	pcl::PointCloud<PointT>::Ptr  &cloud_filtered)
{
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
  	sor.setStddevMulThresh (1.0);
  	sor.setKeepOrganized(true);
  	sor.filter (*cloud_filtered);
	//pcl::PCDWriter writer;
	//writer.write<PointT> ("data_passfil.pcd", *cloud_filtered, false); 
}

void passthroughfilter(const pcl::PointCloud<PointT>::Ptr  &cloud , 
	pcl::PointCloud<PointT>::Ptr  &cloud_filtered)
{
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 1.5);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_filtered);
}

void downsamplefilter(const pcl::PointCloud<PointT>::Ptr  &cloud , 
	pcl::PointCloud<PointT>::Ptr  &cloud_filtered)
{
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*cloud_filtered);
}

void extractinliers(const pcl::PointCloud<PointT>::Ptr  &cloud ,
	const pcl::PointIndices::Ptr &inliers , 
	pcl::PointCloud<PointT>::Ptr  &cloud_filtered,
	bool flag=false)
{
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (flag);
	extract.filter (*cloud_filtered);
}