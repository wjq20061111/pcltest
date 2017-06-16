#include <iostream>
#include <cstring>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "viewer.h"

int main(int argc,char** argv)
{
	 if (argc < 2)
  {
    std::cout << "Need at least two parameters!" << std::endl << std::endl;
    return (-1);
  }
// for(int i=0;i<argc;i++)
// {
// std::cout<<"argv"<<i<<" "<<argv[i]<<std::endl;
// }

	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");
	

	if(strcmp(argv[1], "-rgb") == 0) 
	{
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rgbsrc_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		if (pcl::io::loadPCDFile (argv[filenames[0]], *rgbsrc_cloud) < 0)  
		{
			std::cout << "Error loading point cloud " << std::endl << std::endl;
			return -1;
		}
		rgbpclviewer(rgbsrc_cloud);
	}
	else if(strcmp(argv[1], "-vfh") == 0) 
	{
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhsrc_cloud (new pcl::PointCloud<pcl::VFHSignature308>);
		if (pcl::io::loadPCDFile (argv[filenames[0]], *vfhsrc_cloud) < 0)  
		{
			std::cout << "Error loading point cloud " << std::endl << std::endl;
			return -1;
		}
		VFHviewer(vfhsrc_cloud);
	}
	else if(strcmp(argv[1], "-xyz") == 0) 
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		if (pcl::io::loadPCDFile (argv[filenames[0]], *src_cloud) < 0)  
		{
			std::cout << "Error loading point cloud " << std::endl << std::endl;
			return -1;
		}
		pclviewer(src_cloud);
	}



	// if (pcl::io::loadPCDFile (argv[filenames[0]], *source_cloud) < 0)  {
	// 	std::cout << "Error loading point cloud " << std::endl << std::endl;
	// 	return -1;
	// }

	// pcl::visualization::PCLVisualizer viewer ("Viewer");
	// //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (source_cloud, 255, 255, 255);
	// //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
	// viewer.addPointCloud (source_cloud, "original_cloud");
	// viewer.addCoordinateSystem (1.0, "cloud", 0);
 //  	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
 //  	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
	// while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
	// 	viewer.spinOnce ();
	// }
	return 0;
}