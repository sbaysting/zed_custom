#ifndef VOXELGRID_FILTER_HPP
#define VOXELGRID_FILTER_HPP

pcl::PCLPointCloud2::Ptr voxelgrid_filter(pcl::PCLPointCloud2::Ptr input_cloud);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr approximate_voxelgrid_filter(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud);

#endif
