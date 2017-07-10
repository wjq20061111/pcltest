#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
//#include <thread>
//#include <mutex>
//#include <chrono>
#include <sys/stat.h>
#include <cmath>
#include <limits>  

//kinect2
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>

//opencv 2.4.13
#include <opencv2/opencv.hpp>

//pcl 1.7
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "iai/kinect2_definitions.h"
#include "iai/kinect2_registration.h"

using namespace std;
using namespace cv;

typedef pcl::PointXYZRGBA PointT;


void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud);


class iaiKinect
{
private:
	//calibration+registration
	cv::Size sizeColor, sizeIr, sizeLowRes;
	cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr, cameraMatrixDepth, distortionDepth;
	cv::Mat rotation, translation;
	cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;
	DepthRegistration *depthRegLowRes, *depthRegHighRes;
	 cv::Mat lookupX, lookupY;
	double depthShift;

	//libfreenect2 interface
	libfreenect2::Freenect2 freenect2;
	libfreenect2::Freenect2Device *dev = 0;
	libfreenect2::PacketPipeline *pipeline = 0;
	libfreenect2::Freenect2Device::ColorCameraParams colorParams;
	libfreenect2::Freenect2Device::IrCameraParams irParams;
	libfreenect2::Registration *registration;
	std::string serial = "";

public:
	libfreenect2::SyncMultiFrameListener *listener;
	libfreenect2::FrameMap frames;
	enum Image
	{
		IR_SD = 0,
		IR_SD_RECT,

		DEPTH_SD,
		DEPTH_SD_RECT,
		DEPTH_HD,
		DEPTH_QHD,

		COLOR_SD_RECT,
		COLOR_HD,
		COLOR_HD_RECT,
		COLOR_QHD,
		COLOR_QHD_RECT,

		MONO_HD,
		MONO_HD_RECT,
		MONO_QHD,
		MONO_QHD_RECT,

		COUNT
	};

public:
	iaiKinect():sizeColor(1920, 1080), sizeIr(512, 424), sizeLowRes(sizeColor.width / 2, sizeColor.height / 2), depthShift(0)
	{
		
	}

	~iaiKinect()
	{
		delete depthRegLowRes;
		delete depthRegHighRes;
	}

	void initialize()
	{
		double maxDepth(12.0), minDepth(0.1);
		std::string calib_path;
		calib_path="iai/";
		// if(calib_path.empty() || calib_path.back() != '/')
		// {
		// 	calib_path += '/';
		// }
		//initConfig(bilateral_filter, edge_aware_filter, minDepth, maxDepth);

		initCalibration(calib_path);

		initRegistration(maxDepth);
		
	}

	int startkinect()
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

	void initparam()
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

	void stopkinect()
	{
		dev->stop();
		dev->close();
		delete registration;
		delete listener;
	}

	bool initRegistration(const double maxDepth)
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
		//registration = new libfreenect2::Registration(irParams, colorParams);

		return true;

	}

	void initCalibration(const std::string &calib_path)
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
		cv::initUndistortRectifyMap(cameraMatrixIr, distortionIr, cv::Mat(), cameraMatrixIr, sizeIr, mapType, map1Ir, map2Ir);
		cv::initUndistortRectifyMap(cameraMatrixColor, distortionColor, cv::Mat(), cameraMatrixLowRes, sizeLowRes, mapType, map1LowRes, map2LowRes);
	}

	bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const
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

	bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const
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

	bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const
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

	void processIrDepth(const cv::Mat &depth, std::vector<cv::Mat> &images)
	{
		cv::Mat depthShifted;
		depth.convertTo(depthShifted, CV_16U, 1, depthShift);
		cv::flip(depthShifted, depthShifted, 1);
		cv::remap(depthShifted, images[DEPTH_SD_RECT], map1Ir, map2Ir, cv::INTER_NEAREST);
		depthRegLowRes->registerDepth(depthShifted, images[DEPTH_QHD]);
		//depthRegHighRes->registerDepth(depthShifted, images[DEPTH_HD]);

	}

	void processColor(const cv::Mat &color,std::vector<cv::Mat> &images)
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


  void createLookup(size_t width, size_t height)
  {
    const float fx = 1.0f / cameraMatrixLowRes.at<double>(0, 0);
    const float fy = 1.0f / cameraMatrixLowRes.at<double>(1, 1);
    const float cx = cameraMatrixLowRes.at<double>(0, 2);
    const float cy = cameraMatrixLowRes.at<double>(1, 2);
    float *it;

    lookupY = cv::Mat(1, height, CV_32F);
    it = lookupY.ptr<float>();
    for(size_t r = 0; r < height; ++r, ++it)
    {
      *it = (r - cy) * fy;
    }

    lookupX = cv::Mat(1, width, CV_32F);
    it = lookupX.ptr<float>();
    for(size_t c = 0; c < width; ++c, ++it)
    {
      *it = (c - cx) * fx;
    }
  }

  void createCloud(const cv::Mat &depth, const cv::Mat &color, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud) 
  {
  createLookup(color.cols,color.rows);
    cloud->height = color.rows;
    cloud->width = color.cols;
    cloud->is_dense = false;
    cloud->points.resize(cloud->height * cloud->width);

    const float badPoint = std::numeric_limits<float>::quiet_NaN();

    #pragma omp parallel for
    for(int r = 0; r < depth.rows; ++r)
    {
      pcl::PointXYZRGBA *itP = &cloud->points[r * depth.cols];
      const uint16_t *itD = depth.ptr<uint16_t>(r);
      const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);
      const float y = lookupY.at<float>(0, r);
      const float *itX = lookupX.ptr<float>();

      for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
      {
        register const float depthValue = *itD / 1000.0f;
        // Check for invalid measurements
        if(*itD == 0)
        {
          // not valid
          itP->x = itP->y = itP->z = badPoint;
          itP->rgba = 0;
          continue;
        }
        itP->z = depthValue;
        itP->x = *itX * depthValue;
        itP->y = y * depthValue;
        itP->b = itC->val[0];
        itP->g = itC->val[1];
        itP->r = itC->val[2];
        itP->a = 255;
      }
    }
  }

};

int main (int argc, char** argv)
{
	// Mat depthFrame_cv=imread("depth_raw.jpg",0);
	// Mat ir_cv=imread("ir_raw.jpg",0);
	// Mat color_cv=imread("rgb.jpg");
	// Mat registered_cv=imread("registered.jpg");
	iaiKinect iai;
	std::vector<cv::Mat> images(iaiKinect::COUNT);
	if(iai.startkinect()==-1)
	{
		return -1;
	}
	iai.initialize();

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
pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
	iai.processIrDepth(depthmat,images);
//iai.createLookup(images[iaiKinect::COLOR_QHD_RECT].cols, images[iaiKinect::COLOR_QHD_RECT].rows);
iai.createCloud(images[iaiKinect::DEPTH_QHD],images[iaiKinect::COLOR_QHD_RECT],src_cloud);
std::cout<<images[iaiKinect::DEPTH_QHD].size()<<images[iaiKinect::COLOR_QHD_RECT].size()<<std::endl;
//iai.creatpointcloud(src_cloud,images);

//  cv::imshow("rig",images[iaiKinect::COLOR_QHD_RECT]);
//  cv::imshow("rigd",images[iaiKinect::DEPTH_QHD]*10);
//  cv::waitKey();
pclviewer(src_cloud);
	iai.listener->release(iai.frames);
	iai.stopkinect();
	return 0;



}


void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud)
{
		pcl::visualization::PCLVisualizer viewer ("My example");

	 // Define R,G,B colors for the point cloud
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 255, 255);
	// We add the point cloud to the viewer and pass the color handler
		//viewer.addPointCloud (cloud, cloud_color_handler, "original_cloud");
		viewer.addPointCloud (cloud,"cloud");

		viewer.addCoordinateSystem (1.0, "cloud", 0);
		viewer.setBackgroundColor(5, 5, 5, 0); // Setting background to a dark grey
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

		while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
			viewer.spinOnce ();
		}
}