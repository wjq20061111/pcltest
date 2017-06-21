#ifndef FEATURE_H
#define FEATURE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <flann/flann.h>
#include <flann/io/hdf5.h>
#include <boost/filesystem.hpp>
#include <pcl/features/vfh.h>
#include <vector>
#include <utility>
#include <string>

int nKSearch(pcl::PointCloud<pcl::VFHSignature308>::Ptr &targetvfhs , std::vector<std::pair<int,float> > &kdistance);

#endif // !FEATURE_H