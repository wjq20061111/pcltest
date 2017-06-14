#ifndef VIEWER_H
#define VIEWER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/histogram_visualizer.h>

#include "config.h"


void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud);
void VFHviewer(const pcl::PointCloud<pcl::VFHSignature308>::Ptr &vfhs);

#endif // !VIEWER_H