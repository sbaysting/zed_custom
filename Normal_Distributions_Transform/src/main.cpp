#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>

int
main (int argc, char** argv)
{
  // Loading first scan of room.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("points/1.pcd", *target_cloud_color) == -1)
  {
    PCL_ERROR ("Couldn't read file points/1.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << target_cloud_color->size () << " data points from points/1.pcd" << std::endl;

  // Loading second scan of room from new perspective.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_color (new pcl::PointCloud<pcl::PointXYZRGB>);
  if (pcl::io::loadPCDFile<pcl::PointXYZRGB> ("points/2.pcd", *input_cloud_color) == -1)
  {
    PCL_ERROR ("Couldn't read file points/2.pcd \n");
    return (-1);
  }
  std::cout << "Loaded " << input_cloud_color->size () << " data points from points/2.pcd" << std::endl;

  // Scale point clouds
  Eigen::Matrix4f m;
  m << 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 0.001, 0, 0, 0, 0, 1;
  std::cout << "Scaled matrix: " << std::endl << m << std::endl;
  std::cout << "Scaling point clouds..." << std::endl;
  pcl::transformPointCloud(*target_cloud_color, *target_cloud_color, m);
  pcl::transformPointCloud(*input_cloud_color, *input_cloud_color, m);

  // Save in ASCII format
  std::cout << "Saving clouds in ASCII format..." << std::endl;
  pcl::io::savePCDFileASCII ("1_scaled.pcd", *target_cloud_color);
  pcl::io::savePCDFileASCII ("2_scaled.pcd", *input_cloud_color);
  std::cout << "Clouds saved!" << std::endl;

  // Convert color point clouds to XYZ
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud(*target_cloud_color, *target_cloud);
  pcl::copyPointCloud(*input_cloud_color, *input_cloud);

  // Filtering input scan to roughly 10% of original size to increase speed of registration.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::ApproximateVoxelGrid<pcl::PointXYZRGB> approximate_voxel_filter;
  approximate_voxel_filter.setLeafSize (0.02f, 0.02f, 0.02f);
  approximate_voxel_filter.setInputCloud (input_cloud);
  approximate_voxel_filter.filter (*filtered_cloud);
  std::cout << "Filtered cloud contains " << filtered_cloud->size ()
            << " data points from points/2.pcd" << std::endl;

  // Initializing Normal Distributions Transform (NDT).
  pcl::NormalDistributionsTransform<pcl::PointXYZRGB, pcl::PointXYZRGB> ndt;

  // Setting scale dependent NDT parameters
  // Setting minimum transformation difference for termination condition.
  ndt.setTransformationEpsilon (0.01);
  // Setting maximum step size for More-Thuente line search.
  ndt.setStepSize (0.1);
  //Setting Resolution of NDT grid structure (VoxelGridCovariance).
  ndt.setResolution (1.0);

  // Setting max number of registration iterations.
  ndt.setMaximumIterations (35);

  // Setting point cloud to be aligned.
  std::cout << "Setting input sources" << std::endl;
  ndt.setInputSource (filtered_cloud);
  // Setting point cloud to be aligned to.
  ndt.setInputTarget (target_cloud);

  // Set initial alignment estimate found using robot odometry.
  std::cout << "Setting initial guess" << std::endl;
  Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ ());
  Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
  Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();

  // Calculating required rigid transform to align the input cloud to the target cloud.
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  std::cout << "Calculating transform" << std::endl;
  ndt.align (*output_cloud, init_guess);

  std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
            << " score: " << ndt.getFitnessScore () << std::endl;

  // Transforming unfiltered, input cloud using found transform.
  pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());

  // Saving transformed input cloud.
  pcl::io::savePCDFileASCII ("room_scan2_transformed.pcd", *output_cloud);

  // Initializing point cloud visualizer
  boost::shared_ptr<pcl::visualization::PCLVisualizer>
  viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer_final->setBackgroundColor (0, 0, 0);

  // Coloring and visualizing target cloud (red).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
  target_color (target_cloud, 255, 0, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGB> (target_cloud, target_color, "target cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "target cloud");

  // Coloring and visualizing transformed input cloud (green).
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>
  output_color (output_cloud, 0, 255, 0);
  viewer_final->addPointCloud<pcl::PointXYZRGB> (output_cloud, output_color, "output cloud");
  viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                  1, "output cloud");

  // Starting visualizer
  viewer_final->addCoordinateSystem (1.0, "global");
  viewer_final->initCameraParameters ();

  // Wait until visualizer window is closed.
  while (!viewer_final->wasStopped ())
  {
    viewer_final->spinOnce (100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }

  return (0);
}
