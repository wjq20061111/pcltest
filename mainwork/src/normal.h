#ifndef NORMAL_H
#define NORMAL_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/features/normal_3d.h>

#include "config.h"

void normalcal(const pcl::PointCloud<PointT>::Ptr  &cloud , pcl::PointCloud<pcl::Normal>::Ptr  &normals);

#endif // !NORMAL_H