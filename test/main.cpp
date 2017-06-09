#include "main.h"
#include "viewer.h"
#include "filter.h"
#include "config.h"
#include "segement.h"



int main (int argc, char** argv)
{


	pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);

	pcl::PCDReader reader;
	reader.read ("data2.pcd", *src_cloud);

	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_passfiltered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_samplefiltered (new pcl::PointCloud<PointT>);
	downsamplefilter(src_cloud,cloud_samplefiltered);
	passthroughfilter(cloud_samplefiltered,cloud_passfiltered);
	threedfilter(cloud_passfiltered,cloud_filtered);

	// pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	// pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// findplane(cloud_filtered,inliers, coefficients);
	// if (inliers->indices.size () == 0)
	// {
	// 	PCL_ERROR ("Could not estimate a planar model for the given dataset.");
	// 	return (-1);
	// }

	// pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
	// extractinliers(cloud_filtered,inliers,cloud_plane);
	


	int j = 0;
	std::vector<pcl::PointIndices> cluster_indices;
	ECExtraction(cloud_filtered,cluster_indices);
	pcl::PCDWriter writer;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "seg/" << j << ".pcd";
		writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
		j++;
		
	}
	//pclviewer(cloud_plane);

	return 0;
}
