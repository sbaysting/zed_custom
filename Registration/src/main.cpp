#include <iostream>
#include <sstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "voxelgrid_filter.hpp"
#include "outlier_filter.hpp"
#include "io.hpp"
#include "icp.hpp"
#include "cloud_viewer.hpp"

int main (int argc, char** argv){

	// Parameters

	std::string input_folder = "points/";
	std::string output_folder = "downsampled_points/";

	int clouds = 12;

	// Define point clouds

	pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
	pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp (new pcl::PointCloud<pcl::PointXYZRGB>);
	std::ostringstream input_filename;
	std::ostringstream output_filename;

	// Loop over the clouds and filter them

	for(int i = 1; i <= clouds; i++){

		// Set filename
		input_filename << input_folder << i << ".pcd";

		// Read the point cloud in PointCloud2 format
		cloud = readPointCloud2(input_filename.str());

		// Filter using VoxelGrid
		cloud_filtered = voxelgrid_filter(cloud);

		// Filter outliers
		cloud_filtered = outlier_filter(cloud_filtered);

		// If its the first iteration, this is going to be the base cloud. Set it as such
		if(i == 1){
			final_cloud = convertToXYZRGB(cloud_filtered);
		} else { // If not the first iteration, run ICP on the base cloud and the input cloud
			temp = convertToXYZRGB(cloud_filtered);
			final_cloud = stitchWithICP(temp, final_cloud);
		}

		/*// Write new PCD file
		output_filename << output_folder << i << ".pcd";
		writePointCloud2(output_filename.str(), cloud_filtered);
		*/

		input_filename.str("");
		input_filename.clear();
		output_filename.str("");
		output_filename.clear();

		std::cout << std::endl;

	} // End for loop

	displayPointCloud(final_cloud);

	return (0);
}
