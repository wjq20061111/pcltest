#ifndef FILTER_H
#define FILTER_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include "config.h"

void threedfilter(const pcl::PointCloud<PointT>::Ptr  &cloud , pcl::PointCloud<PointT>::Ptr  &cloud_filtered);
void passthroughfilter(const pcl::PointCloud<PointT>::Ptr  &cloud , pcl::PointCloud<PointT>::Ptr  &cloud_filtered);
void downsamplefilter(const pcl::PointCloud<PointT>::Ptr  &cloud , pcl::PointCloud<PointT>::Ptr  &cloud_filtered);
void extractinliers(const pcl::PointCloud<PointT>::Ptr  &cloud ,const pcl::PointIndices::Ptr &inliers , pcl::PointCloud<PointT>::Ptr  &cloud_filtered);

#endif // !FILTER_H