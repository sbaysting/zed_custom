#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "icp.hpp"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitchWithICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match){

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setInputSource(cloud_to_rotate);
	icp.setInputTarget(cloud_to_match);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return Final;

}

