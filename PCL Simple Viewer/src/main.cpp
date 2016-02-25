
// Standard Libraries
#include <iostream>
#include <string>

// PCL Libraries
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char** argv){

	// Get filename
	std::string file;
	std::cout << "Filename to display: ";
	std::cin >> file;

	// Define a point cloud pointer
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	// Load PCD point cloud file
    pcl::io::loadPCDFile (file, *cloud);
	// Define a cloud viewer
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	// Show point cloud
	viewer.showCloud(cloud);

	// Loop to do things while the point cloud viewer is still on
	while (!viewer.wasStopped())
	{
	}	

}
