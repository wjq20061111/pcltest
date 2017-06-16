#include "viewer.h"
#include "segement.h"
#include "config.h"
#include <iostream>
#include <sstream>

int main (int argc, char** argv)
{
	pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
	pcl::PCDReader reader;	
	reader.read ("data2.pcd", *src_cloud);
	passthroughfilter(src_cloud,src_cloud);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	
	pcl::SACSegmentation<PointT> seg;
  	// Optional
	seg.setOptimizeCoefficients (true);
  	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_LMEDS);
	seg.setDistanceThreshold (0.01);
	seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
	seg.setEpsAngle(3.14/180*10); 
	seg.setInputCloud (src_cloud);
	seg.segment (*inliers, *coefficients);

	pcl::PointCloud<PointT>::Ptr temp_cloud_plane (new pcl::PointCloud<PointT>);
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (src_cloud);
	extract.setIndices (inliers);
	extract.setNegative (false);

    	// Get the points associated with the planar surface
	extract.filter (*temp_cloud_plane);

	pcl::visualization::PCLVisualizer viewer ("My example");
int viewport=0;
	viewer.createViewPort (0, 0, 0.5, 1, viewport);
	viewer.addPointCloud (src_cloud,"cloud",viewport);
	viewer.addCoordinateSystem (0.1, "cloud", 0);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (temp_cloud_plane,255,0,0);
	viewer.addPointCloud (temp_cloud_plane,color_handler, "target",viewport);
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
	

//pcl::SACSegmentation<PointT> seg;
  	// Optional
	//seg.setOptimizeCoefficients (true);
  	// Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC   );
	seg.setDistanceThreshold (0.01);
	seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
	seg.setEpsAngle(3.14/180*10); 
	seg.setInputCloud (src_cloud);
	seg.segment (*inliers, *coefficients);

	//pcl::PointCloud<PointT>::Ptr temp_cloud_plane (new pcl::PointCloud<PointT>);
//	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (src_cloud);
	extract.setIndices (inliers);
	extract.setNegative (false);

    	// Get the points associated with the planar surface
	extract.filter (*temp_cloud_plane);


viewer.createViewPort (0.5, 0, 1, 1, viewport);
	viewer.addPointCloud (src_cloud,"cloud2",viewport);
	viewer.addCoordinateSystem (0.1, "cloud2", 0);
	viewer.setBackgroundColor(0.3, 0.3, 0.3, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud2");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler2 (temp_cloud_plane,255,0,0);
	viewer.addPointCloud (temp_cloud_plane,color_handler2, "target2",viewport);
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target2");
	

	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}

}

