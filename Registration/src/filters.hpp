#ifndef FILTERS_HPP
#define FILTERS_HPP

// Statistical Outlier Filter, returns a filtered cloud pointer
pcl::PointCloud<pcl::PointXYZRGB> outlier_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

// Approximate VoxelGrid Filter (works like a box grid filter), returns a filtered cloud pointer
pcl::PointCloud<pcl::PointXYZRGB> voxelgrid_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

// Approximate VoxelGrid Filter (works like a box grid filter), returns a filtered cloud pointer. Uses small leaf size for merging point clouds
pcl::PointCloud<pcl::PointXYZRGB> merge_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

#endif
