#ifndef FILTERS_HPP
#define FILTERS_HPP

// Statistical Outlier Filter, returns a filtered cloud pointer
pcl::PointCloud<pcl::PointXYZRGB>::Ptr outlier_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

// Approximate VoxelGrid Filter (works like a box grid filter), returns a filtered cloud pointer
pcl::PointCloud<pcl::PointXYZRGB>::Ptr voxelgrid_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

#endif
