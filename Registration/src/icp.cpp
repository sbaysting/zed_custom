#include <iostream>
#include <Eigen/Geometry>
#include <pcl/registration/icp.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "icp.hpp"


pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitchWithICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match){

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setTransformationEpsilon (1e-12);
	icp.setEuclideanFitnessEpsilon (1e-8);
	icp.setInputSource(cloud_to_rotate);
	icp.setInputTarget(cloud_to_match);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*Final);
	std::cout << std::endl << "ICP has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return Final;

}

Eigen::Matrix4f getICPTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match){

	pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
	icp.setTransformationEpsilon (1e-12);
	icp.setEuclideanFitnessEpsilon (1e-8);
	icp.setInputSource(cloud_to_rotate);
	icp.setInputTarget(cloud_to_match);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGB>);
	icp.align(*Final);
	std::cout << std::endl << "ICP has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return icp.getFinalTransformation();

}

