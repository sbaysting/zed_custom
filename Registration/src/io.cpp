// Standard
#include <iostream>

// Eigen
#include <Eigen/Geometry>

// PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

// Custom
#include "io.hpp"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToXYZRGB(pcl::PCLPointCloud2::Ptr input){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*input, *output);
	return output;

}

pcl::PCLPointCloud2::Ptr convertToPointCloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input){

	pcl::PCLPointCloud2::Ptr output (new pcl::PCLPointCloud2 ());
	pcl::toPCLPointCloud2(*input, *output);
	return output;

}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readXYZRGB(std::string filename){

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *output);
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr scaleCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	Eigen::Matrix4f m; // Transformation matrix to scale the point cloud coordinates to 1/1000 (convert to meters)
    m << 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 1;
	std::cout << std::endl << "Scaling point clouds..." << std::endl;
	pcl::transformPointCloud(*cloud, *cloud, m);
	
	return cloud;

}
