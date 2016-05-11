// Standard
#include <iostream>

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/normal_3d.h>
#include <pcl/impl/point_types.hpp>

// Custom
#include "stitching_algorithms.hpp"
#include "io.hpp"

// Estimate surface normals from point cloud data, concatenate with XYZRGB data and return the cloud
pcl::PointCloud<pcl::PointXYZRGBNormal> estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	// Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud (cloud);

	// Create an empty kdtree representation, and pass it to the normal estimation object.
	// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
	ne.setSearchMethod (tree);

	// Output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

	// Use all neighbors in a sphere of radius 3cm
	ne.setRadiusSearch (0.03);

	// Compute the features
	ne.compute (*cloud_normals);

	// Concatenate fields into one cloud
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::concatenateFields(*cloud, *cloud_normals, *cloud_with_normals); 

	return *cloud_with_normals;

}

// Iterative Closest Point (ICP) 3D point cloud stitching algorithm, returns a transformed point cloud pointer
pcl::PointCloud<pcl::PointXYZRGB> stitchWithICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match){

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

	return *Final;

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

// Uses ICP to calculate a rigid transformation with a SVD transformation initial guess and returns the transformation matrix, no edits are made to the point cloud
Eigen::Matrix4f getICPTransformationWithPTPGuess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate_no_normals, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match_no_normals){

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_to_rotate (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_to_match (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	Eigen::Matrix4f guess;
	// Estimate normals
	*cloud_to_rotate = estimateNormals(cloud_to_rotate_no_normals);
	*cloud_to_match = estimateNormals(cloud_to_match_no_normals);
	// Resize clouds for size to match
	adjustCloudSize(cloud_to_match,cloud_to_rotate);
	// Estimate transformation using SVD
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>::Ptr TESVD (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal>());
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBNormal,pcl::PointXYZRGBNormal>::Matrix4 transformation2;
	TESVD->estimateRigidTransformation (*cloud_to_rotate,*cloud_to_match,transformation2);
	std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << std::endl;
	printf ("\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (0,0), transformation2 (0,1), transformation2 (0,2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", transformation2 (1,0), transformation2 (1,1), transformation2 (1,2));
	printf ("    | %6.3f %6.3f %6.3f | \n", transformation2 (2,0), transformation2 (2,1), transformation2 (2,2));
	printf ("\n");
	printf ("t = < %0.3f, %0.3f, %0.3f >\n", transformation2 (0,3), transformation2 (1,3), transformation2 (2,3));
	// Use ICP to get the final transformation using the point to plane estimation as a guess
	pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	icp.setTransformationEpsilon (1e-12);
	icp.setEuclideanFitnessEpsilon (1e-8);
	icp.setInputSource(cloud_to_rotate);
	icp.setInputTarget(cloud_to_match);
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (0.05);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (50);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr Final (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	icp.setTransformationEstimation(TESVD);
	icp.align(*Final);
	std::cout << std::endl << "ICP has converged:" << icp.hasConverged() << " score: " <<
	icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;

	return icp.getFinalTransformation();

}



