#ifndef IO_HPP
#define IO_HPP

pcl::PCLPointCloud2::Ptr readPointCloud2(std::string filename);

bool writePointCloud2(std::string filename, pcl::PCLPointCloud2::Ptr cloud);

#endif
