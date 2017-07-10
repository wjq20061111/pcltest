#include "viewer.h"
#include "filter.h"
#include "normal.h"
#include "segement.h"
#include "VFH.h"
#include "config.h"
#include "feature.h"

#include "interface.h"

bool mycomp(const std::pair<int,float> &ap ,const  std::pair<int,float> &bp)
{
	return ap.second<bp.second;
}

int main (int argc, char** argv)
{
	pcl::PCDWriter writer;
	pcl::PCDReader reader;
	pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
	
	getKinectcloud(src_cloud);
	writer.write<PointT> ("src_cloud.pcd", *src_cloud, false);

	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	pcl::PointCloud<PointT>::Ptr cloud_passfiltered (new pcl::PointCloud<PointT>);
	passthroughfilter(src_cloud,cloud_passfiltered,'z',0,2);
	threedfilter(cloud_passfiltered,cloud_filtered);

	pcl::visualization::PCLVisualizer viewer ("My example");
	std::vector<int> viewport;
	int viewportm;
	int viewportc=0;int viewportmax=0;
	int numk=5;
	for(int i=0;i<numk;i++)
	{
		for(int j=0;j<numk;j++)
		{
			viewer.createViewPort (i*1.0/numk, j*1.0/numk, (1+i)*1.0/numk, (1+j)*1.0/numk, viewportm);
			viewport.push_back(viewportm);
			viewportmax++;
		}
	}

	viewer.removeAllPointClouds();
	viewportc=0;
	viewer.addPointCloud (cloud_filtered,"cloud",viewport.at(viewportc));
	viewer.addCoordinateSystem (0.1, "cloud", 0);
	viewer.setBackgroundColor(0.4, 0.4, 0.4, viewport.at(viewportc)); 
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	viewportc++;


	std::vector <pcl::PointIndices> clusters;
	pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;
	RegionGrowingSeg(cloud_filtered, clusters,colored_cloud);

	int allpoints=cloud_filtered->points.size();
	int planeindex=-1;
	Eigen::Vector4f centroid;
	for (int i=0;i<clusters.size();i++)
	{
		if(clusters[i].indices.size()>allpoints*0.05)
		{
			std::cout<<clusters[i].indices.size()<<std::endl;
			pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT>);
			extractinliers(cloud_filtered,
				boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices( clusters[i] ) ),
				cloud_plane);
			if(viewportc<viewportmax)
			{
				std::stringstream ssc;
				ssc<<"cloud"<<viewportc;
				viewer.addPointCloud (cloud_plane,ssc.str(),viewport.at(viewportc));
				viewer.addCoordinateSystem (0.1, ssc.str(), 0);
				viewer.setBackgroundColor(0.3, 0.3, 0.3, viewport.at(viewportc)); 
				viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ssc.str());
				viewportc++;
			}
			pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients);
			pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices);
			
			pcl::SACSegmentation<PointT> seg;
  	// Optional
			seg.setOptimizeCoefficients (true);
  	// Mandatory
			seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
			seg.setMethodType (pcl::SAC_RANSAC);
			seg.setDistanceThreshold (0.01);
			seg.setInputCloud (cloud_plane);
			seg.segment (*plane_inliers, *plane_coefficients);

			std::cerr << "Model coefficients: " << plane_coefficients->values[0] << " " 
			<< plane_coefficients->values[1] << " "
			<< plane_coefficients->values[2] << " " 
			<< plane_coefficients->values[3] << std::endl;
			if((std::fabs(plane_coefficients->values[1])>0.8)&&
				(std::fabs(plane_coefficients->values[0])<0.2)&&
				(std::fabs(plane_coefficients->values[2])<0.2))
			{
				if(planeindex==-1)
				{		
					planeindex=i;

					pcl::compute3DCentroid (*cloud_plane, centroid);
				}
				else if(clusters[i].indices.size()>clusters[planeindex].indices.size())
				{	
					planeindex=i;

					pcl::compute3DCentroid (*cloud_plane, centroid);
				}
			}
		//pcl::compute3DCentroid (*cloud_plane, centroid);
		//std::cout<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<" "<<centroid[3]<<std::endl;
extractinliers(cloud_filtered,
		boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices( clusters.at(i)) ),
		cloud_filtered,true,true);
		}
	}
// pcl::compute3DCentroid (*cloud_plane, centroid);
 //	std::cout<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<" "<<centroid[3]<<std::endl;
	// pcl::PointCloud<PointT>::Ptr cut_plane (new pcl::PointCloud<PointT>);
	// 	extractinliers(cloud_filtered,
	// 	boost::shared_ptr<pcl::PointIndices>(new pcl::PointIndices( clusters.at(planeindex)) ),
	// 	cut_plane,true);
	pcl::PointCloud<PointT>::Ptr cloud_trueplane (new pcl::PointCloud<PointT>);
	threedfilter(cloud_filtered,cloud_filtered);
	passthroughfilter(cloud_filtered,cloud_trueplane,'y',centroid[1]-0.2,centroid[1]+0.2);
	
	// writer.write<PointT> ("data_p.pcd", *cloud_trueplane, false); //*
	// reader.read ("data_p.pcd", *cloud_trueplane);
		

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::PointCloud<PointT>::Ptr copy_src_cloud (new pcl::PointCloud<PointT>());
	//downsamplefilter(cloud_trueplane, copy_src_cloud);
	pcl::copyPointCloud(*cloud_trueplane, *copy_src_cloud); 
	
		if(viewportc<viewportmax)
	{
		std::stringstream ssc;
		ssc<<"cloud"<<viewportc;
		viewer.addPointCloud (copy_src_cloud,ssc.str(),viewport.at(viewportc));
		viewer.addCoordinateSystem (0.1, ssc.str(), 0);
		viewer.setBackgroundColor(0.1, 0.1, 0.1, viewport.at(viewportc)); 
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ssc.str());
		viewportc++;
	}

//   	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud (copy_src_cloud);

	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.01); // 2cm
	ec.setMinClusterSize (200);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (copy_src_cloud);
	ec.extract (cluster_indices);

	std::vector<std::pair<int,float> > kdistance;
	pcl::PointCloud<PointT>::Ptr target (new pcl::PointCloud<PointT> ());
	int j = 0;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
			cloud_cluster->points.push_back (copy_src_cloud->points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "seg/0_"<< j << ".pcd";
		writer.write<PointT> (ss.str (), *cloud_cluster, false); //*
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
		calVFH(cloud_cluster,vfhs);
		std::stringstream ssv;
		ssv << "seg/0_"<< j << "_vfh.pcd";
		writer.write<pcl::VFHSignature308> (ssv.str (), *vfhs, false);
		
		if(viewportc<viewportmax)
		{
			viewer.addPointCloud (cloud_cluster,ss.str(),viewport.at(viewportc));
			viewer.addCoordinateSystem (0.1, ss.str(), 0);
			viewer.setBackgroundColor(0.3, 0.3, 0.3, viewport.at(viewportc)); 
			viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
			viewportc++;
		}
		std::cout<<std::endl;
		std::cout<<"model "<<j<<std::endl;
		
		nKSearch(vfhs,kdistance,j);
		j++;
	}
	sort(kdistance.begin(),kdistance.end(),mycomp);
	std::map<int,int> countmap;
	for(int j=0;j<5;j++)
	{
		std::cout<<kdistance.at(j).first<<" with distance "<<kdistance.at(j).second<<std::endl;
		countmap[kdistance.at(j).first]++;
	}
	int posstarget=-1;
	int targetindex=0;
	for(std::map<int,int>::iterator it=countmap.begin();it!=countmap.end();it++)
	{
		std::cout<<it->first<<" "<<it->second<<std::endl;
		if(it->second>posstarget)
		{
			posstarget=it->second;
			targetindex=it->first;
		}
	}
	std::stringstream sst;
	sst << "seg/0_"<< targetindex << ".pcd";
	reader.read (sst.str(), *target);
	if(viewportc<viewportmax)
	{
		viewer.addPointCloud (target,"target",viewport.at(viewportc));
		viewer.addCoordinateSystem (0.1, "target", 0);
		viewer.setBackgroundColor(0.7, 0.7, 0.7, viewport.at(viewportc)); 
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
		viewportc++;
	}
	 pcl::compute3DCentroid (*target, centroid);
 	std::cout<<centroid[0]<<" "<<centroid[1]<<" "<<centroid[2]<<" "<<centroid[3]<<std::endl;
	while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
		viewer.spinOnce (100);
		//boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
	return 0;
}