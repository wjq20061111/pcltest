#include "main.h"
#include "config.h"
#include "viewer.h"
#include "filter.h"
#include "normal.h"
#include "segement.h"



int main (int argc, char** argv)
{
	pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);

	//replace with interface
	pcl::PCDReader reader;
	reader.read ("data2.pcd", *src_cloud);

	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_passfiltered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_samplefiltered (new pcl::PointCloud<PointT>);
	//VoxelGrid+PassThrough+StatisticalOutlierRemoval
	downsamplefilter(src_cloud,cloud_samplefiltered);
	passthroughfilter(cloud_samplefiltered,cloud_passfiltered);
	threedfilter(cloud_passfiltered,cloud_filtered);

	//segement based on normal
	std::vector <pcl::PointIndices> clusters;
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
	pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
	RegionGrowingSeg(cloud_filtered, clusters,colored_cloud);

	//the biggest cluster is the plane
	pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
	extractinliers(cloud_filtered,
		boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices( clusters[0] ) ),
		cloud_plane);
	//calculate the plane with ax+by+cz+d=0
	pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
	findplane(cloud_plane,plane_inliers, plane_coefficients);

	std::cerr << "Model coefficients: " << plane_coefficients->values[0] << " " 
	<< plane_coefficients->values[1] << " "
	<< plane_coefficients->values[2] << " " 
	<< plane_coefficients->values[3] << std::endl;

	double plane_b=plane_coefficients->values[1];
	double plane_d=plane_coefficients->values[3];
	double plane_dis=-plane_d/plane_b;

	//cut the plane±0.2
	pcl::PointCloud<PointT>::Ptr cloud_trueplane (new pcl::PointCloud<PointT>);
	passthroughfilter(cloud_filtered,cloud_trueplane,'y',plane_dis-0.2,plane_dis+0.2);

	//pclviewer(cloud_trueplane);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<PointT> seg;
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointCloud<PointT>::Ptr temp_cloud_plane (new pcl::PointCloud<PointT> ());
	pcl::PointCloud<PointT>::Ptr cloud_f (new pcl::PointCloud<PointT>);
	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setMaxIterations (100);
	seg.setDistanceThreshold (0.02);

	int i=0, nr_points = (int) cloud_trueplane->points.size ();
	while (cloud_trueplane->points.size () > 0.3 * nr_points)
	{
    	// Segment the largest planar component from the remaining cloud
		seg.setInputCloud (cloud_trueplane);
		seg.segment (*inliers, *coefficients);
		if (inliers->indices.size () == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

    	// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud (cloud_trueplane);
		extract.setIndices (inliers);
		extract.setNegative (false);

    	// Get the points associated with the planar surface
		extract.filter (*temp_cloud_plane);
		std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    	// Remove the planar inliers, extract the rest
		extract.setNegative (true);
		extract.filter (*cloud_f);
		*cloud_trueplane = *cloud_f;
	}

//pclviewer(cloud_trueplane);

  	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (cloud_trueplane);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud_trueplane);
	ec.extract (cluster_indices);

	pcl::PCDWriter writer;
	pcl::PointCloud<PointT>::Ptr target (new pcl::PointCloud<PointT> ());
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (cloud_trueplane->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "seg/" << j << ".pcd";
		writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
    		j++;
    		if(j==4)
	reader.read (ss.str (), *target);
	}

	//eye candy
	pcl::visualization::PCLVisualizer viewer ("My example");

	viewer.addPointCloud (cloud_filtered,"cloud");
	viewer.addCoordinateSystem (1.0, "cloud", 0);
	viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	pcl::visualization::PointCloudColorHandlerCustom<PointT> color_handler (target,255,0,0);
	viewer.addPointCloud (target,color_handler, "target");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce ();
	}

	return 0;
}
