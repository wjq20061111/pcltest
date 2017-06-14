#include "viewer.h"
#include "filter.h"
#include "normal.h"
#include "segement.h"
#include "VFH.h"
#include "config.h"
#include "capture.h"

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing);

bool captureflag=0;

int main (int argc, char** argv)
{
	
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	std::string serial = "";

	if(freenect2.enumerateDevices() == 0)
	{
		std::cout << "no device connected!" << std::endl;
		return -1;
	}
	if (serial == "")
	{
		serial = freenect2.getDefaultDeviceSerialNumber();
	}
	pipeline = new libfreenect2::CpuPacketPipeline();//other choices omitted
	
	if(pipeline)
	{
		dev = freenect2.openDevice(serial, pipeline);
	}
	else 
	{
		dev = freenect2.openDevice(serial);
	}
	if(dev == 0)
	{
		std::cout << "failure opening device!" << std::endl;
		return -1;
	}
	else
	{
		std::cout << "device standby" << std::endl;
	}

	bool enable_rgb = true;
	bool enable_depth = true;
	int types = 0;
	if (enable_rgb)
		types |= libfreenect2::Frame::Color;
	if (enable_depth)
		types |= libfreenect2::Frame::Ir | libfreenect2::Frame::Depth;
	libfreenect2::SyncMultiFrameListener listener(types);
	libfreenect2::FrameMap frames;
	dev->setColorFrameListener(&listener);
	dev->setIrAndDepthFrameListener(&listener);
	std::cout << "listener ready" << std::endl;

	if (enable_rgb && enable_depth)
	{
		if (!dev->start())//true if ok, false if error.
			return -1;
	}
	else
	{
		if (!dev->startStreams(enable_rgb, enable_depth))
			return -1;
	}
	std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
	std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;
	
	
	// libfreenect2::Freenect2Device::IrCameraParams myirpara;
	// float cameraMatrix[]={3.9421859733566441e+02, 0., 2.4894837623969377e+02, 0.,
	// 	3.9579406429885239e+02, 2.0348622077734936e+02, 0., 0., 1. };
	// float distortionCoefficients[]={2.2063090417612469e-01, -6.0566383520878908e-01,
	// 	-9.0874099498715472e-03, 8.7100803182754425e-05,4.2608216028665052e-01};
	// myirpara.fx=cameraMatrix[0];
	// myirpara.fy=cameraMatrix[4];
	// myirpara.cx=cameraMatrix[2];
	// myirpara.cy=cameraMatrix[5];
	// myirpara.k1=distortionCoefficients[0];
	// myirpara.k2=distortionCoefficients[1];
	// myirpara.k3=distortionCoefficients[4];
	// myirpara.p1=distortionCoefficients[2];
	// myirpara.p2=distortionCoefficients[3];
	// dev->setIrCameraParams(myirpara);

	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),depth2rgb(1920, 1080 + 2, 4);
	
	Mat rgbmat,depthmat,irmat;
	Mat depthmatUndistorted,rgbd,rgbd2;
	std::cout << "while start" << std::endl;	
	bool protonect_shutdown = false; 
	size_t framemax = 500;
	size_t framecount = 0;

	int j=0;
	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	pcl::visualization::PCLVisualizer viewer ("My example");

	while(!protonect_shutdown && (framemax == (size_t)-1 || framecount < framemax))
	{
		if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
		{
			std::cout << "timeout!" << std::endl;
			return -1;
		}
		libfreenect2::Frame *rgb = frames[libfreenect2::Frame::Color];
		libfreenect2::Frame *ir = frames[libfreenect2::Frame::Ir];
		libfreenect2::Frame *depth = frames[libfreenect2::Frame::Depth];
		
		// cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		// cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
		// cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);
		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);
		// cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
		// cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		// cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

		float cx=0,cy=0,fx=1,fy=1;
		float xu,yu;
		pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::PointType p;
		//p.x = 1; p.y = 2; p.z = 3;
		// for(int xi=0;xi<512;xi++)
		// {
		// 	for(int yi=0;yi<424;yi++)
		// 	{
		// 		xu=(xi+0.5-cx)/fx;
		// 		yu=(yi+0.5-cy)/fy;

		// 		p.x = xu*undistorted.data[512*yi+xi];
		// 		p.y= yu*undistorted.data[512*yi+xi];
		// 		p.z = undistorted.data[512*yi+xi];
		// //RGB = registered[512*yi+xi];
		// 	cloud->push_back(p);
		// 	}
		// }
		for(int r=0;r<512;r++)
		{
			for(int c=0;c<424;c++)
			{
			float point3d[3];
			float bgr;
			registration->getPointXYZRGB(&undistorted,&registered,r,c,point3d[0],point3d[1],point3d[2],bgr);
			p.x = point3d[0];
			p.y = point3d[1];
			p.z = point3d[2];
			uint8_t *colorp = reinterpret_cast<uint8_t*>(&bgr);
			p.b = colorp[0];
			p.g = colorp[1];
			p.r = colorp[2];
			src_cloud->push_back(p);
			}
		}

		pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_passfiltered (new pcl::PointCloud<PointT>);
		passthroughfilter(src_cloud,cloud_passfiltered);
		threedfilter(cloud_passfiltered,cloud_filtered);

		if(framecount==0)
		{
			viewer.addPointCloud (cloud_filtered,"cloud");
			viewer.addCoordinateSystem (1.0, "cloud", 0);
			viewer.setBackgroundColor(0.3, 0.3, 0.3, 0); // Setting background to a dark grey
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		}
		else
			viewer.updatePointCloud (cloud_filtered,"cloud");
		viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL);

		//while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce (100);
			boost::this_thread::sleep (boost::posix_time::microseconds (100000));
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
		listener.release(frames);
	}

	dev->stop();
	dev->close();
	delete registration;
	return 0;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,
	void* nothing
	)
{
	if(event.getKeySym() == "space" && event.keyDown()){
		std::cout<<"get space key"<<std::endl;
		if(captureflag==0)
			captureflag=1;
	}
}