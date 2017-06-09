#ifndef VIEWER_H
#define VIEWER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "config.h"


void pclviewer(const pcl::PointCloud<PointT>::Ptr  &cloud);

#endif // !VIEWER_H