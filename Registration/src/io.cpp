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
#include "filters.hpp"

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

template<typename PointT>
bool writeXYZRGB(std::string filename, pcl::PointCloud<PointT> cloud){

	pcl::io::savePCDFile (filename, cloud, false);
	return true;

}

pcl::PointCloud<pcl::PointXYZRGB> scaleCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

	Eigen::Matrix4f m; // Transformation matrix to scale the point cloud coordinates to 1/1000 (convert to meters)
    m << 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 1;
	std::cout << std::endl << "Scaling point clouds..." << std::endl;
	pcl::transformPointCloud(*cloud, *cloud, m);
	
	return *cloud;

}

pcl::PointCloud<pcl::PointXYZRGB> cloudMerge(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_merge, pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_cloud){

	*static_cloud += *cloud_to_merge;
	*static_cloud = merge_filter(static_cloud);
	return *static_cloud;

}

// Take two clouds and upscale the smaller one to make it the same number of points
void adjustCloudSize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2){

	int cur_size = 0;
	if(cloud1->points.size() > cloud2->points.size()){
		//writeXYZRGB("cloud2_orig.pcd",*cloud2);
		cur_size = cloud2->points.size();
		std::cout << "Current cloud 1 size:" << cloud1->points.size() << std::endl;
		std::cout << "Current cloud 2 size:" << cloud2->points.size() << std::endl;
		cloud2->width = cloud1->width;
		cloud2->height = cloud1->height;
		std::cout << "Resizing cloud 2..." << std::endl;
		cloud2->points.resize(cloud2->width * cloud2->height);
		//writeXYZRGB("cloud2.pcd",*cloud2);
		std::cout << "New cloud 2 size:" << cloud2->points.size() << std::endl;
	} if(cloud2->points.size() > cloud1->points.size()){
		//writeXYZRGB("cloud1_orig.pcd",*cloud1);
		cur_size = cloud1->points.size();
		std::cout << "Current cloud 1 size:" << cloud1->points.size() << std::endl;
		std::cout << "Current cloud 2 size:" << cloud2->points.size() << std::endl;
		cloud1->width = cloud2->width;
		cloud1->height = cloud2->height;
		std::cout << "Resizing cloud 1..." << std::endl;
		cloud1->points.resize(cloud1->width * cloud1->height);
		//writeXYZRGB("cloud1.pcd",*cloud1);
		std::cout << "New cloud 1 size:" << cloud1->points.size() << std::endl;
	}

}
