#include <iostream>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

int main (int argc, char** argv)
{
	typedef pcl::PointXYZRGB PointT;
	pcl::search::Search <PointT>::Ptr tree = boost::shared_ptr<pcl::search::Search<PointT> > (new pcl::search::KdTree<PointT>);

	pcl::PointCloud <PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
	if ( pcl::io::loadPCDFile <PointT> ("data2.pcd", *cloud) == -1 )
	{
		std::cout << "Cloud reading f./readingailed." << std::endl;
		return (-1);
	}

	pcl::IndicesPtr indices (new std::vector <int>);
	pcl::PassThrough<PointT> pass;
	pass.setInputCloud (cloud);
	pass.setFilterFieldName ("z");
	pass.setFilterLimits (0.0, 3.0);
	pass.filter (*indices);

	pcl::RegionGrowingRGB<PointT> reg;
	reg.setInputCloud (cloud);
	reg.setIndices (indices);
	reg.setSearchMethod (tree);
	reg.setDistanceThreshold (10);
	reg.setPointColorThreshold (6);
	reg.setRegionColorThreshold (5);
	reg.setMinClusterSize (600);

	std::vector <pcl::PointIndices> clusters;
	reg.extract (clusters);

	pcl::PointCloud <PointT>::Ptr colored_cloud = reg.getColoredCloud ();
	pcl::visualization::CloudViewer viewer ("Cluster viewer");
	viewer.showCloud (colored_cloud);
	while (!viewer.wasStopped ())
	{
		boost::this_thread::sleep (boost::posix_time::microseconds (100));
	}

	return (0);
}