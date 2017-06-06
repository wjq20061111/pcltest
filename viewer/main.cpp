#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc,char** argv)
{
	typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
	CloudType::Ptr source_cloud (new CloudType);
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

	if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
		std::cout << "Error loading point cloud " << std::endl << std::endl;
		return -1;
	}

	pcl::visualization::PCLVisualizer viewer ("Viewer");
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
	//viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
	viewer.addPointCloud (source_cloud, "original_cloud");
	viewer.addCoordinateSystem (1.0, "cloud", 0);
  	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}
	return 0;
}