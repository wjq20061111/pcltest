#include "segement.h"
#include "viewer.h"
#include "filter.h"
#include "config.h"

int main (int argc, char** argv)
{
	pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);

	//replace with interface
	for(int fi=0;fi<12;fi++)
	{
	std::stringstream ssf;
	ssf<< "cap/orig" << fi << ".pcd";
	pcl::PCDReader reader;
	reader.read (ssf.str(), *src_cloud);

	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_passfiltered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_passfiltered2 (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_samplefiltered (new pcl::PointCloud<PointT>);
	//VoxelGrid+PassThrough+StatisticalOutlierRemoval
	//downsamplefilter(src_cloud,cloud_samplefiltered);
	passthroughfilter(src_cloud,cloud_passfiltered,'y',-0.2,0.2);
	passthroughfilter(cloud_passfiltered,cloud_passfiltered,'z',0,1.2);
	threedfilter(cloud_passfiltered,cloud_filtered);
//pclviewer(cloud_filtered);
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr temp_cloud_plane (new pcl::PointCloud<PointT> ());
	pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.01);

	int i=0, nr_points = (int) cloud_filtered->points.size ();
	while (cloud_filtered->points.size () > 0.5 * nr_points)
	{
    	// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_filtered);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

    	// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (cloud_filtered);
		extract.setIndices (inliers);
		extract.setNegative (false);

    	// Get the points associated with the planar surface
		extract.filter (*temp_cloud_plane);
//pclviewer(temp_cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_filtered->points.size () << " data points." << std::endl;

    	// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_filtered = *cloud_f;
	}
passthroughfilter(cloud_filtered,cloud_filtered,'z',0,1.2);
//pclviewer(cloud_filtered);

  	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_filtered);
	ec.extract (cluster_indices);

	pcl::PCDWriter writer;
	pcl::PointCloud<PointT>::Ptr target (new pcl::PointCloud<PointT> ());
	int j = 0;
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
		ss << "seg2/" <<fi<<"_"<< j << ".pcd";
		writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
    		j++;
	}
}
	return 0;
}