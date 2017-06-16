#include "viewer.h"

void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud)
{
		pcl::visualization::PCLVisualizer viewer ("My example");

		viewer.addPointCloud (cloud,"cloud");
		viewer.addCoordinateSystem (1.0, "cloud", 0);
		viewer.setBackgroundColor(0.3, 0.3, 0.3, 0); // Setting background to a dark grey
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce (100);
			//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		}
}

void VFHviewer(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &vfhs)
{
	pcl::visualization::PCLHistogramVisualizer hv;
	hv.addFeatureHistogram(*vfhs,308);
	hv.spin();
}

void calViewerXY(int k,double (&step)[2])
{
	int y_s = (int)floor (sqrt ((double)k));
  	int x_s = y_s + (int)ceil ((k / (double)y_s) - y_s);
  	double x_step = (double)(1 / (double)x_s);
  	double y_step = (double)(1 / (double)y_s);
  	step[0]=x_step; step[1]=y_step;
}