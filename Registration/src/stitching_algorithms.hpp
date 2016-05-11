#ifndef STITCHING_HPP
#define STITCHING_HPP

// Estimate surface normals from point cloud data, concatenate with XYZRGB data and return the cloud
pcl::PointCloud<pcl::PointXYZRGBNormal> estimateNormals(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// Iterative Closest Point (ICP) 3D point cloud stitching algorithm, returns a transformed point cloud pointer
pcl::PointCloud<pcl::PointXYZRGB> stitchWithICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match);

// Uses ICP to calculate a rigid transformation and returns the transformation matrix, no edits are made to the point cloud
Eigen::Matrix4f getICPTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match);

// Uses ICP to calculate a rigid transformation with an initial point to plane guess and returns the transformation matrix, no edits are made to the point cloud
Eigen::Matrix4f getICPTransformationWithPTPGuess(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match);

#endif
