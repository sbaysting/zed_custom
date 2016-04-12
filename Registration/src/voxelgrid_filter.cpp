#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include "voxelgrid_filter.hpp"

pcl::PCLPointCloud2::Ptr voxelgrid_filter(pcl::PCLPointCloud2::Ptr input_cloud){

	pcl::PCLPointCloud2::Ptr output_cloud (new pcl::PCLPointCloud2 ());

	std::cerr << std::endl << "PointCloud before voxelgrid filtering: " << input_cloud->width * input_cloud->height 
	   << " data points (" << pcl::getFieldsList (*input_cloud) << ").";

	// Create the filtering object
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (input_cloud);
	sor.setLeafSize (0.01f, 0.01f, 0.01f);
	sor.filter (*output_cloud);

	std::cerr << std::endl << "PointCloud after voxelgrid filtering: " << output_cloud->width * output_cloud->height 
	   << " data points (" << pcl::getFieldsList (*output_cloud) << ").";

	return output_cloud;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr approximate_voxelgrid_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud){

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
