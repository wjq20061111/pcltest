#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/vfh.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/visualization/pcl_plotter.h>
//#include <pcl/visualization/pcl_visualizer.h>

int main (int argc, char** argv)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());

  //... read, pass in or create a point cloud with normals ...
	pcl::io::loadPCDFile ("table_scene_mug_stereo_textured.pcd", *cloud) ;

	pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
	ne.setMaxDepthChangeFactor(0.02f);
	ne.setNormalSmoothingSize(10.0f);
	ne.setInputCloud(cloud);
	ne.compute(*normals);

  // Create the VFH estimation class, and pass the input dataset+normals to it
	pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
	vfh.setInputCloud (cloud);
	vfh.setInputNormals (normals);
  // alternatively, if cloud is of type PointNormal, do vfh.setInputNormals (cloud);

  // Create an empty kdtree representation, and pass it to the FPFH estimation object.
  // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	vfh.setSearchMethod (tree);

  // Output datasets
	pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());

  // Compute the features
	vfh.compute (*vfhs);

  // vfhs->points.size () should be of size 1*
	pcl::visualization::PCLPlotter hv();
	hv.addFeatureHistogram(*vfhs,308);
	hv.spin();
}