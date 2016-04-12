#ifndef IO_HPP
#define IO_HPP

pcl::PointCloud<pcl::PointXYZRGB>::Ptr convertToXYZRGB(pcl::PCLPointCloud2::Ptr input);

pcl::PCLPointCloud2::Ptr convertToPointCloud2(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr readXYZRGB(std::string filename);

pcl::PCLPointCloud2::Ptr readPointCloud2(std::string filename);

bool writePointCloud2(std::string filename, pcl::PCLPointCloud2::Ptr cloud);

#endif
