#include "main.h"

int main(int argc,char** argv)
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
	
	libfreenect2::Registration* registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
	libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4),depth2rgb(1920, 1080 + 2, 4);
	
	Mat rgbmat,depthmat,irmat;
	Mat depthmatUndistorted,rgbd,rgbd2;
	std::cout << "while start" << std::endl;	
	bool protonect_shutdown = false; 
	size_t framemax = 1;
	size_t framecount = 0;

	ofstream ofile;
	ofile.open("DATA.txt");
	if (!ofile)
	{
		cout << "open failed\n";
	}
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
		
		cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
		cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
		cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);

		//cv::imshow("rgb", rgbmat);
		//cv::imshow("ir", irmat / 4500.0f); 
		//cv::imshow("depth", depthmat / 4500.0f); 

		// imwrite("rgb.jpg", rgbmat);
		// imwrite("ir_raw.jpg", irmat);
		// imwrite("depth_raw.jpg", depthmat);

		registration->apply(rgb, depth, &undistorted, &registered, true, &depth2rgb);

		cv::Mat(undistorted.height, undistorted.width, CV_32FC1, undistorted.data).copyTo(depthmatUndistorted);
		cv::Mat(registered.height, registered.width, CV_8UC4, registered.data).copyTo(rgbd);
		cv::Mat(depth2rgb.height, depth2rgb.width, CV_32FC1, depth2rgb.data).copyTo(rgbd2);

		cv::imshow("undistorted", depthmatUndistorted / 4500.0f);
		//cv::imshow("registered", rgbd);
		//cv::imshow("depth2RGB", rgbd2 / 4500.0f);

		// imwrite("undistorted.jpg", depthmatUndistorted);
		// imwrite("registered.jpg", rgbd);
		// imwrite("depth2RGB.jpg", rgbd2);
		int r,c;
		int count=0;
		for(r=0;r<512;r++)
		{
			for(c=0;c<424;c++)
			{
			float point3d[3];
			registration->getPointXYZ(&undistorted,r,c,point3d[0],point3d[1],point3d[2]);
			ofile<<r<<" "<<c<<" "<<point3d[0]<<" "<<point3d[1]<<" "<<point3d[2]<<" "<<endl;
			}
		}
		float pers=count*1.0/512/424;
		std::cout<<pers<<std::endl;

		framecount++;
		std::cout << framecount<<std::endl;

		int key = cv::waitKey(0);
		listener.release(frames);
	}

	ofile.close();

	dev->stop();
	dev->close();
	delete registration;
	//std::cout << "All clear!" << std::endl;
	return 0;
}


	