#ifndef ICP_HPP
#define ICP_HPP

pcl::PointCloud<pcl::PointXYZRGB>::Ptr stitchWithICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match);

Eigen::Matrix4f getICPTransformation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_rotate, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_match);

#endif
