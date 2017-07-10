#include "viewer.h"
#include "filter.h"
#include "normal.h"
#include "segement.h"
#include "config.h"
#include "capture.h"
#include "interface.h"

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing);

bool captureflag=0;

int main (int argc, char** argv)
{
	int j=0;
	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	
	iaiKinect iai;
	std::vector<cv::Mat> images(iaiKinect::COUNT);
	iai.initialize();
	iai.startkinect();
	int framecount=0;
	pcl::visualization::PCLVisualizer viewer ("My example");
			
	while(framecount<500)
	{
		pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
		if (!iai.listener->waitForNewFrame(iai.frames, 10*1000)) // 10 sconds
		{
			std::cout << "timeout!" << std::endl;
			return -1;
		}
		libfreenect2::Frame *rgb = iai.frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = iai.frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = iai.frames[libfreenect2::Frame::Depth];
		Mat rgbmat,depthmat,irmat;
		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
		cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);

		iai.processColor(rgbmat,images);
		iai.processIrDepth(depthmat,images,src_cloud);

		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_passfiltered (new pcl::PointCloud<PointT>);
		passthroughfilter(src_cloud,cloud_passfiltered,'z',0,1.0);
		threedfilter(cloud_passfiltered,cloud_filtered);
		if(framecount==0)
		{
			viewer.removeAllPointClouds();
			viewer.addPointCloud (cloud_filtered,"cloud");
			viewer.addCoordinateSystem (0.1, "cloud", 0);
			viewer.setBackgroundColor(0.4, 0.4, 0.4); 
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
			viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);
		}
		else
		{
			viewer.updatePointCloud (cloud_filtered,"cloud");
		}
		
		//while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce (1000);
			//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
		//}

		if(captureflag==1)
		{
			std::stringstream ss;
			ss << "cap/orig" << j << ".pcd";
			writer.write<PointT> (ss.str (), *cloud_filtered, false);
			captureflag=0;
			std::cout<<"saved"<<ss.str()<<std::endl;
			j++;
		}

		framecount++;
		std::cout << framecount<<std::endl;
		//int key = cv::waitKey(0);

		iai.listener->release(iai.frames);
	}
	iai.stopkinect();
	return 0;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing
	)
{
	if(event.getKeySym() == "space" && event.keyDown()){
		//std::cout<<"get space key"<<std::endl;
		if(captureflag==0)
			captureflag=1;
	}
}