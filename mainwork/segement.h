#ifndef SEGEMENT_H
#define SEGEMENT_H

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

#include "config.h"
#include "filter.h"

void findplane(const pcl::PointCloud<PointT>::Ptr  &cloud ,pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients);
void ECExtraction(const pcl::PointCloud<PointT>::Ptr  &cloud_filtered , std::vector<pcl::PointIndices> cluster_indices);

#endif // !SEGEMENT_H