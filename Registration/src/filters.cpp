// Standard
#include <iostream>

// PCL
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>

// Custom
#include "filters.hpp"

// Statistical Outlier Filter, returns a filtered cloud pointer
pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::cerr << std::endl << "PointCloud before outlier filtering: " << input_cloud->width * input_cloud->height 
	   << " data points (" << pcl::getFieldsList (*input_cloud) << ").";

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud (input_cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*output_cloud);

	std::cerr << std::endl << "PointCloud after outlier filtering: " << output_cloud->width * output_cloud->height 
	   << " data points (" << pcl::getFieldsList (*output_cloud) << ").";

	return output_cloud;

}

// Approximate VoxelGrid Filter (works like a box grid filter), returns a filtered cloud pointer
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelgrid_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

	std::cerr << std::endl << "PointCloud before approximate voxelgrid filtering: " << input_cloud->width * input_cloud->height 
	   << " data points (" << pcl::getFieldsList (*input_cloud) << ").";

	// Create the filtering object
	pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> sor;
	sor.setInputCloud (input_cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*output_cloud);

	std::cerr << std::endl << "PointCloud after approximate voxelgrid filtering: " << output_cloud->width * output_cloud->height 
	   << " data points (" << pcl::getFieldsList (*output_cloud) << ").";

	return output_cloud;

}
