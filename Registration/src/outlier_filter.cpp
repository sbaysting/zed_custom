#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "outlier_filter.hpp"

pcl::PCLPointCloud2::Ptr outlier_filter(pcl::PCLPointCloud2::Ptr input_cloud){

	pcl::PCLPointCloud2::Ptr output_cloud (new pcl::PCLPointCloud2 ());

	std::cerr << std::endl << "PointCloud before outlier filtering: " << input_cloud->width * input_cloud->height 
	   << " data points (" << pcl::getFieldsList (*input_cloud) << ").";

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (input_cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*output_cloud);

	std::cerr << std::endl << "PointCloud after outlier filtering: " << output_cloud->width * output_cloud->height 
	   << " data points (" << pcl::getFieldsList (*output_cloud) << ").";

	return output_cloud;

}
