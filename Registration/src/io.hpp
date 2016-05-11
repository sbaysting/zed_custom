#ifndef IO_HPP
#define IO_HPP

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToXYZRGB(pcl::PCLPointCloud2::Ptr input);

pcl::PCLPointCloud2::Ptr convertToPointCloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readXYZRGB(std::string filename);

pcl::PCLPointCloud2::Ptr readPointCloud2(std::string filename);

bool writePointCloud2(std::string filename, pcl::PCLPointCloud2::Ptr cloud);

template<typename PointT>
bool writeXYZRGB(std::string filename, pcl::PointCloud<PointT> cloud);

pcl::PointCloud<pcl::PointXYZRGB> scaleCloud (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

pcl::PointCloud<pcl::PointXYZRGB> cloudMerge(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_to_merge, pcl::PointCloud<pcl::PointXYZRGB>::Ptr static_cloud);

// Take two clouds and upscale the smaller one to make it the same number of points
void adjustCloudSize(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud2);

#endif
