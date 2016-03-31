
// Standard Libraries
#include <iostream>
#include <string>

// PCL Libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "cloud_viewer.hpp"

void displayPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input){

	// Define a cloud viewer
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	// Show point cloud
	viewer.showCloud(input);

	// Loop to do things while the point cloud viewer is still on
	while (!viewer.wasStopped())
	{
	}	

}
