#ifndef OCCUPANCY_GRID_MAP_H
#define OCCUPANCY_GRID_MAP_H

#include <Eigen/Dense>
#include <boost/shared_array.hpp>
#include <fstream>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <ros/ros.h>

class OccupancyGridMap {
 public:
  void loadOccupancyGridMap(const std::string &file_name, nav_msgs::OccupancyGrid &occupancy_grid);
  void occupancyGridToPCD(const nav_msgs::OccupancyGrid &occupancyGrid, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);
  void performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                  Eigen::Matrix4f &transformation_matrix,
                  float leaf_size);
};

#endif// OCCUPANCY_GRID_MAP_H
