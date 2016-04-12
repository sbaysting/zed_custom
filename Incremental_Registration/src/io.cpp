#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "io.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToXYZRGB(pcl::PCLPointCloud2::Ptr input){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*input, *output);
	return output;

}

pcl::PCLPointCloud2::Ptr readPointCloud2(std::string filename){

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	std::cout << std::endl << "Reading " << filename;

	// Fill in the cloud data
	pcl::PCDReader reader;
	// Replace the path below with the path where you saved your file
	reader.read (filename, *cloud);

	return cloud;

}

bool writePointCloud2(std::string filename, pcl::PCLPointCloud2::Ptr cloud){

	pcl::PCDWriter writer;
	std::cout << std::endl << "Writing " << filename;
	writer.write (filename, *cloud, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);
	
	return true;

}
