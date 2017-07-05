#ifndef INTERFACE_H
#define INTERFACE_H

#include <iostream>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include <sys/stat.h>

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

class iaiKinect
{
private:
	//calibration+registration
	cv::Size sizeColor, sizeIr, sizeLowRes;
	cv::Mat cameraMatrixColor, distortionColor, cameraMatrixLowRes, cameraMatrixIr, distortionIr, cameraMatrixDepth, distortionDepth;
	cv::Mat rotation, translation;
	cv::Mat map1Color, map2Color, map1Ir, map2Ir, map1LowRes, map2LowRes;
	DepthRegistration *depthRegLowRes, *depthRegHighRes;
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
	iaiKinect();

	~iaiKinect();

	void initialize();

	int startkinect();

	void initparam();

	void stopkinect();

	bool initRegistration(const double maxDepth);

	void initCalibration(const std::string &calib_path);

	bool loadCalibrationFile(const std::string &filename, cv::Mat &cameraMatrix, cv::Mat &distortion) const;
	
	bool loadCalibrationPoseFile(const std::string &filename, cv::Mat &rotation, cv::Mat &translation) const;

	bool loadCalibrationDepthFile(const std::string &filename, double &depthShift) const;

	void processIrDepth(const cv::Mat &depth, std::vector<cv::Mat> &images,pcl::PointCloud<PointT>::Ptr &src_cloud);

	void processColor(const cv::Mat &color,std::vector<cv::Mat> &images);
};

bool getKinectcloud(pcl::PointCloud<PointT>::Ptr &src_cloud);

#endif // !INTERFACE_H