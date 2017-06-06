#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int main (int argc, char** argv)
{
  typedef pcl::PointXYZ PointT;
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

// Fill in the cloud data
  cloud->width = 5;
  cloud->height = 1;
  cloud->points.resize (cloud->width * cloud->height);

  for (size_t i = 0; i < cloud->points.size (); ++i)
  {
    cloud->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    cloud->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }

  std::cerr << "Cloud before filtering: " << std::endl;
  for (size_t i = 0; i < cloud->points.size (); ++i)
    std::cerr << "    " << cloud->points[i].x << " " 
  << cloud->points[i].y << " " 
  << cloud->points[i].z << std::endl;

// Create the filtering object
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 0.3);
pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
    std::cerr << "    " << cloud_filtered->points[i].x << " " 
  << cloud_filtered->points[i].y << " " 
  << cloud_filtered->points[i].z << std::endl;

  return (0);
}