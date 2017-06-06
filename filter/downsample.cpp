#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZ PointT;

int main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read ("data_inliers.pcd", *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";


  // writer.write ("table_scene_lms400_downsampled.pcd", *cloud_filtered, 
  //        Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
pcl::PointCloud<PointT>::Ptr cloud_v (new pcl::PointCloud<PointT>);
fromPCLPointCloud2 (*cloud_filtered, *cloud_v);

  //  pcl::PCDWriter writer;
  // writer.write("dsdata_inliers.pcd",*cloud_v,false);

  pcl::visualization::PCLVisualizer viewer ("Viewer");
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
  viewer.addPointCloud (cloud_v, "original_cloud");
  viewer.addCoordinateSystem (1.0, "cloud", 0);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }

  return (0);
}