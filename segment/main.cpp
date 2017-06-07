#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
	typedef pcl::PointXYZ PointT;
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

	pcl::io::loadPCDFile("data_deskwithcup_f.pcd",*cloud);

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
	pcl::SACSegmentation<PointT> seg;
  // Optional
	seg.setOptimizeCoefficients (true);
  // Mandatory
	seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);
	seg.setAxis(Eigen::Vector3f (0.0, 0.0, 1.0));
	seg.setEpsAngle(3.14/180*10); 
	seg.setInputCloud (cloud);
	seg.segment (*inliers, *coefficients);


	if (inliers->indices.size () == 0)
	{
		PCL_ERROR ("Could not estimate a planar model for the given dataset.");
		return (-1);
	}

  // std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
  //                                     << coefficients->values[1] << " "
  //                                     << coefficients->values[2] << " " 
  //                                     << coefficients->values[3] << std::endl;

  // std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;
  // for (size_t i = 0; i < inliers->indices.size (); ++i)
  //   std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
  //                                              << cloud->points[inliers->indices[i]].y << " "
  //                                              << cloud->points[inliers->indices[i]].z << std::endl;
	pcl::PointCloud<PointT>::Ptr inlier_cloud(new pcl::PointCloud<PointT>);
	PointT pointi;
	for (size_t i = 0; i < inliers->indices.size (); ++i)
	{
		pointi.x=cloud->points[inliers->indices[i]].x;
		pointi.y=cloud->points[inliers->indices[i]].y;
		pointi.z=cloud->points[inliers->indices[i]].z;
		inlier_cloud->push_back(pointi);
	}
	pcl::visualization::PCLVisualizer viewer ("segment");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_color_handler (cloud, 255, 255, 255);
	viewer.addPointCloud (cloud, cloud_color_handler, "original_cloud");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> inlier_cloud_color_handler (inlier_cloud, 230, 20, 20); // Red
	viewer.addPointCloud (inlier_cloud, inlier_cloud_color_handler, "inlier_cloud");
	//viewer.addPointCloud (cloud, "original_cloud");
	viewer.addCoordinateSystem (1.0, "cloud", 0);
  	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "inlier_cloud");
	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
}
return (0);
}