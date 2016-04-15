#ifndef STITCHING_HPP
#define STITCHING_HPP

// Iterative Closest Point (ICP) 3D point cloud stitching algorithm, returns a transformed point cloud pointer
pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitchWithICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match);

// Uses ICP to calculate a rigid transformation and returns the transformation matrix, no edits are made to the point cloud
Eigen::Matrix4f getICPTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match);

#endif
