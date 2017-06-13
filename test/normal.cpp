#include "normal.h"

void normalcal(const pcl::PointCloud<PointT>::Ptr  &cloud , 
	pcl::PointCloud<pcl::Normal>::Ptr  &normals)
{
	if(cloud->isOrganized())
	{
	pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
}
else
{
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	pcl::NormalEstimation<PointT, pcl::Normal> normal_estimator;
	normal_estimator.setSearchMethod (tree);
	normal_estimator.setInputCloud (cloud);
	normal_estimator.setKSearch (50);
	normal_estimator.compute (*normals);
}
}