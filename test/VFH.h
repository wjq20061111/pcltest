#ifndef VFH_H
#define VFH_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/vfh.h>
#include <pcl/features/integral_image_normal.h>

#include "config.h"
#include "normal.h"

void calVFH(pcl::PointCloud<PointT>::Ptr &cloud,pcl::PointCloud<pcl::VFHSignature308>::Ptr &vfhs);

#endif // !VFH_H