#ifndef MAIN_H
#define MAIN_H


#include <iostream>
#include <string>

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

using namespace std;
using namespace cv;

#endif // !MAIN_H