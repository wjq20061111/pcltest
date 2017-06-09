#include "viewer.h"

void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud)
{
		pcl::visualization::PCLVisualizer viewer ("My example");

		viewer.addPointCloud (cloud,"cloud");
		viewer.addCoordinateSystem (0.1, "cloud", 0);
		viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce ();
		}
}
