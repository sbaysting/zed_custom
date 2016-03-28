#include <iostream>
#include <sstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "voxelgrid_filter.hpp"
#include "outlier_filter.hpp"
#include "io.hpp"

int
main (int argc, char** argv)
{
  pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
  pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
  std::ostringstream input_filename;
  std::ostringstream output_filename;

  // Parameters

  std::string input_folder = "points/";
  std::string output_folder = "downsampled_points/";

  int clouds = 12;

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

	  // Write new PCD file
	  output_filename << output_folder << i << ".pcd";
	  writePointCloud2(output_filename.str(), cloud_filtered);

	  input_filename.str("");
	  input_filename.clear();
	  output_filename.str("");
	  output_filename.clear();

	  std::cout << std::endl;

  } // End for loop

  return (0);
}
