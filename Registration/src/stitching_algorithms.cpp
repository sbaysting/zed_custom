// Standard
#include <iostream>

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/registration/icp.h>

// Custom
#include "stitching_algorithms.hpp"

// Iterative Closest Point (ICP) 3D point cloud stitching algorithm, returns a transformed point cloud pointer
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

// Uses ICP to calculate a rigid transformation and returns the transformation matrix, no edits are made to the point cloud
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

