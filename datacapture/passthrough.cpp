#include <iostream>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int  main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointT;
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

  pcl::PCDReader reader;
 // pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>), cloud_f (new pcl::PointCloud<PointT>);
  reader.read ("data_filted.pcd", *cloud);

  // Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.5);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

pcl::PCDWriter writer;
writer.write<PointT> ("data_passfil.pcd", *cloud_filtered, false); 
  // pcl::visualization::PCLVisualizer viewer ("Viewer");
  // //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
  // //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
  // viewer.addPointCloud (cloud_filtered, "original_cloud");
  // viewer.addCoordinateSystem (1.0, "cloud", 0);
  // viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  // //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
  // while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
  //   viewer.spinOnce ();
  // }

  return (0);
}