#include "interface.h"

typedef pcl::PointXYZRGBA PointT;

void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud);


iaiKinect::iaiKinect():sizeColor(1920, 1080), sizeIr(512, 424), sizeLowRes(sizeColor.width / 2, sizeColor.height / 2), depthShift(0)
{
}

iaiKinect::~iaiKinect()
{
	delete depthRegLowRes;
	delete depthRegHighRes;
}

void iaiKinect::initialize()
{
	double maxDepth(12.0), minDepth(0.1);
	std::string calib_path;
	calib_path="../iai/";
	//change if needed
	initCalibration(calib_path);
	initRegistration(maxDepth);
}

int iaiKinect::startkinect()
{
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

	listener=new libfreenect2::SyncMultiFrameListener(types);
	dev->setColorFrameListener(listener);
	dev->setIrAndDepthFrameListener(listener);
	std::cout << "listener ready" << std::endl;
	colorParams = dev->getColorCameraParams();
	irParams = dev->getIrCameraParams();
	registration = new libfreenect2::Registration(irParams, colorParams);
	initparam();
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
}

void iaiKinect::initparam()
{
	cameraMatrixColor = cv::Mat::eye(3, 3, CV_64F);
	distortionColor = cv::Mat::zeros(1, 5, CV_64F);

	cameraMatrixColor.at<double>(0, 0) = colorParams.fx;
	cameraMatrixColor.at<double>(1, 1) = colorParams.fy;
	cameraMatrixColor.at<double>(0, 2) = colorParams.cx;
	cameraMatrixColor.at<double>(1, 2) = colorParams.cy;
	cameraMatrixColor.at<double>(2, 2) = 1;

	cameraMatrixIr = cv::Mat::eye(3, 3, CV_64F);
	distortionIr = cv::Mat::zeros(1, 5, CV_64F);

	cameraMatrixIr.at<double>(0, 0) = irParams.fx;
	cameraMatrixIr.at<double>(1, 1) = irParams.fy;
	cameraMatrixIr.at<double>(0, 2) = irParams.cx;
	cameraMatrixIr.at<double>(1, 2) = irParams.cy;
	cameraMatrixIr.at<double>(2, 2) = 1;

	distortionIr.at<double>(0, 0) = irParams.k1;
	distortionIr.at<double>(0, 1) = irParams.k2;
	distortionIr.at<double>(0, 2) = irParams.p1;
	distortionIr.at<double>(0, 3) = irParams.p2;
	distortionIr.at<double>(0, 4) = irParams.k3;

	cameraMatrixDepth = cameraMatrixIr.clone();
	distortionDepth = distortionIr.clone();

	rotation = cv::Mat::eye(3, 3, CV_64F);
	translation = cv::Mat::zeros(3, 1, CV_64F);
}

void iaiKinect::stopkinect()
{
	dev->stop();
	dev->close();
	delete registration;
	delete listener;
}

bool iaiKinect::initRegistration(const double maxDepth)
{
	DepthRegistration::Method reg;
	reg = DepthRegistration::CPU;
	depthRegLowRes = DepthRegistration::New(reg);
	depthRegHighRes = DepthRegistration::New(reg);
	int device=0;
	if(!depthRegLowRes->init(cameraMatrixLowRes, sizeLowRes, cameraMatrixDepth, sizeIr, distortionDepth, rotation, translation, 0.3f, maxDepth, device) ||
		!depthRegHighRes->init(cameraMatrixColor, sizeColor, cameraMatrixDepth, sizeIr, distortionDepth, rotation, translation, 0.3f, maxDepth, device))
	{
		delete depthRegLowRes;
		delete depthRegHighRes;
		return false;
	}

	return true;
}

void iaiKinect::initCalibration(const std::string &calib_path)
{
	std::string calibPath = calib_path;

	struct stat fileStat;
	bool calibDirNotFound = stat(calibPath.c_str(), &fileStat) != 0 || !S_ISDIR(fileStat.st_mode);
	if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_COLOR, cameraMatrixColor, distortionColor))
	{
		std::cout<<"using sensor defaults for color intrinsic parameters."<<std::endl;
	}

	if(calibDirNotFound || !loadCalibrationFile(calibPath + K2_CALIB_IR, cameraMatrixDepth, distortionDepth))
	{
		std::cout<<"using sensor defaults for ir intrinsic parameters."<<std::endl;
	}

	if(calibDirNotFound || !loadCalibrationPoseFile(calibPath + K2_CALIB_POSE, rotation, translation))
	{
		std::cout<<"using defaults for rotation and translation."<<std::endl;
	}

	if(calibDirNotFound || !loadCalibrationDepthFile(calibPath + K2_CALIB_DEPTH, depthShift))
	{
		std::cout<<"using defaults for depth shift."<<std::endl;
		depthShift = 0.0;
	}
	cameraMatrixLowRes = cameraMatrixColor.clone();
	cameraMatrixLowRes.at<double>(0, 0) /= 2;
	cameraMatrixLowRes.at<double>(1, 1) /= 2;
	cameraMatrixLowRes.at<double>(0, 2) /= 2;
	cameraMatrixLowRes.at<double>(1, 2) /= 2;
	const int mapType = CV_16SC2;
	cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixColor, sizeColor, mapType, map1Color, map2Color);
	cv::initUndistortRectifyMap(cameraMatrixDepth, distortionDepth, cv::Mat(), cameraMatrixDepth, sizeIr, mapType, map1Ir, map2Ir);
	cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);
}

bool iaiKinect::loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
{
	cv::FileStorage fs;
	if(fs.open(filename, cv::FileStorage::READ))
	{
		fs[K2_CALIB_CAMERA_MATRIX] >> cameraMatrix;
		fs[K2_CALIB_DISTORTION] >> distortion;
		fs.release();
	}
	else
	{
			//OUT_ERROR("can't open calibration file: " << filename);
		return false;
	}
	return true;
}

bool iaiKinect::loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
{
	cv::FileStorage fs;
	if(fs.open(filename, cv::FileStorage::READ))
	{
		fs[K2_CALIB_ROTATION] >> rotation;
		fs[K2_CALIB_TRANSLATION] >> translation;
		fs.release();
	}
	else
	{
			//OUT_ERROR("can't open calibration pose file: " << filename);
		return false;
	}
	return true;
}

bool iaiKinect::loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
{
	cv::FileStorage fs;
	if(fs.open(filename, cv::FileStorage::READ))
	{
		fs[K2_CALIB_DEPTH_SHIFT] >> depthShift;
		fs.release();
	}
	else
	{
			//OUT_ERROR("can't open calibration depth file: " << filename);
		return false;
	}
	return true;
}

void iaiKinect::processIrDepth(const cv::Mat &depth, std::vector<cv::Mat> &images,pcl::PointCloud<PointT>::Ptr &src_cloud)
{
	cv::Mat depthShifted;
	depth.convertTo(depthShifted, CV_16U, 1, depthShift);
	cv::flip(depthShifted, depthShifted, 1);
	cv::remap(depthShifted, images[DEPTH_SD_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
	depthRegLowRes->registerDepth(depthShifted, images[DEPTH_QHD],images[COLOR_QHD_RECT],src_cloud);
	//depthRegHighRes->registerDepth(depthShifted, images[DEPTH_HD]);

}

void iaiKinect::processColor(const cv::Mat &color,std::vector<cv::Mat> &images)
{
	cv::Mat tmp;
	cv::flip(color, tmp, 1);	
	cv::cvtColor(tmp, images[COLOR_HD], CV_BGRA2BGR);
	 //cv::cvtColor(tmp, images[COLOR_HD], CV_RGBA2BGR);

	cv::resize(images[COLOR_HD], images[COLOR_QHD], sizeLowRes, 0, 0, cv::INTER_AREA);
	cv::remap(images[COLOR_HD], images[COLOR_QHD_RECT], map1LowRes, map2LowRes, cv::INTER_AREA);
	cv::cvtColor(images[COLOR_QHD], images[MONO_QHD], CV_BGR2GRAY);
	cv::cvtColor(images[COLOR_QHD_RECT], images[MONO_QHD_RECT], CV_BGR2GRAY);
}

bool getKinectcloud(pcl::PointCloud<PointT>::Ptr &src_cloud)
{
	iaiKinect iai;
	std::vector<cv::Mat> images(iaiKinect::COUNT);
	iai.initialize();
	iai.startkinect();
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
	iai.listener->release(iai.frames);
	iai.stopkinect();
	return 0;
}



// int main (int argc, char** argv)
// {

// 	iaiKinect iai;
// 	std::vector<cv::Mat> images(iaiKinect::COUNT);
// 	iai.initialize();
// 	iai.startkinect();

// 	if (!iai.listener->waitForNewFrame(iai.frames, 10*1000)) // 10 sconds
// 	{
// 		std::cout << "timeout!" << std::endl;
// 		return -1;
// 	}
// 	libfreenect2::Frame *rgb = iai.frames[libfreenect2::Frame::Color];
// 	libfreenect2::Frame *ir = iai.frames[libfreenect2::Frame::Ir];
// 	libfreenect2::Frame *depth = iai.frames[libfreenect2::Frame::Depth];
// 	Mat rgbmat,depthmat,irmat;
// 	cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data).copyTo(rgbmat);
// 	cv::Mat(depth->height, depth->width, CV_32FC1, depth->data).copyTo(depthmat);
// 	cv::Mat(ir->height, ir->width, CV_32FC1, ir->data).copyTo(irmat);

// 	iai.processColor(rgbmat,images);
// 	pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
// 	iai.processIrDepth(depthmat,images,src_cloud);
// 	//iai.creatpointcloud(src_cloud,images);
// 	// cv::imshow("rig",images[iaiKinect::COLOR_QHD_RECT]);
// 	// cv::imshow("rigd",images[iaiKinect::DEPTH_QHD]);
// 	// cv::waitKey();
// 	pclviewer(src_cloud);
// 	iai.listener->release(iai.frames);
// 	iai.stopkinect();
// 	return 0;
// }
