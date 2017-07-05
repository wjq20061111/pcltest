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
	pcl::PointCloud<PointT>::Ptr  &cloud_filtered,
	char field,
	float limitdown,
	float limitup)
{
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	switch(field)
	{
		case 'x' : pass.setFilterFieldName ("x");break;
		case 'y' : pass.setFilterFieldName ("y");break;
		case 'z' : pass.setFilterFieldName ("z");break;
		default : pass.setFilterFieldName ("z");break;
	}	
	pass.setFilterLimits (limitdown, limitup);
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
	bool Negative_flag,
	bool Organized_flag)
{
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers);
	extract.setNegative (Negative_flag);
	extract.setKeepOrganized(Organized_flag);
	extract.filter (*cloud_filtered);
}