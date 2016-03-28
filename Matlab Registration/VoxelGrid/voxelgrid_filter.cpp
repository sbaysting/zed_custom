#include <iostream>
#include <sstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

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
  std::cout << "Reading " << input_filename.str() << std::endl;

  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read (input_filename.str(), *cloud); // Remember to download the file first!

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (10.0f, 10.0f, 10.0f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  pcl::PCDWriter writer;
  output_filename << output_folder << i << ".pcd";
  std::cout << "Writing " << output_filename.str() << std::endl;
  writer.write (output_filename.str(), *cloud_filtered, Eigen::Vector4f::Zero (), Eigen::Quaternionf::Identity (), false);

  input_filename.str("");
  input_filename.clear();
  output_filename.str("");
  output_filename.clear();

  } // End for loop

  return (0);
}
