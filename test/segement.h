#ifndef SEGEMENT_H
#define SEGEMENT_H

#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/search/search.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include "config.h"
#include "filter.h"
#include "normal.h"

void findplane(const pcl::PointCloud<PointT>::Ptr  &cloud ,pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients);
void ECExtraction(const pcl::PointCloud<PointT>::Ptr  &cloud_filtered , std::vector<pcl::PointIndices> &cluster_indices);
void RegionGrowingSeg(const pcl::PointCloud<PointT>::Ptr  &cloud , std::vector <pcl::PointIndices> &clusters,pcl::PointCloud <pcl::PointXYZRGB>::Ptr &colored_cloud);
void RegionGrowingRGBSeg(const pcl::PointCloud<PointT>::Ptr  &cloud , std::vector <pcl::PointIndices> &clusters,pcl::PointCloud <pcl::PointXYZRGB>::Ptr &colored_cloud);
void calEuclideanClusterExtraction(const pcl::PointCloud<PointT>::Ptr  &cloud , std::vector <pcl::PointIndices> &cluster_indices);

#endif // !SEGEMENT_H