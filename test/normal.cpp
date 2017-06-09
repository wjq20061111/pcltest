#include "normal.h"

void normalcal(const pcl::PointCloud<PointT>::Ptr  &cloud , pcl::PointCloud<pcl::Normal>::Ptr  &normals)
{
	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);
}