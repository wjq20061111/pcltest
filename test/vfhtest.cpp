#include "filter.h"
#include "VFH.h"
#include "viewer.h"
#include "config.h"
#include <iostream>
#include <sstream>

int main (int argc, char** argv)
{
	
	pcl::PCDReader reader;
	pcl::PCDWriter writer;

	//for(int fi=0;fi<12;fi++)
	//{
		pcl::PointCloud<PointT>::Ptr src_cloud (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr filter_cloud (new pcl::PointCloud<PointT>);

		//std::stringstream ssf;
		//ssf<< "cup/" << fi<<"_0.pcd";
		//reader.read (ssf.str(), *src_cloud);
		reader.read ("milk.pcd", *src_cloud);
		downsamplefilter(src_cloud,filter_cloud);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs (new pcl::PointCloud<pcl::VFHSignature308> ());
		calVFH(filter_cloud,vfhs);

		VFHviewer(vfhs);

		// std::stringstream ss;
		// ss<< "cup/" << fi<<"_0_vfh.pcd";
		// writer.write<pcl::VFHSignature308>(ss.str(),*vfhs,false);
		writer.write<pcl::VFHSignature308>("milk_filted_vfh.pcd",*vfhs,false);
	//}

	return 0;
}