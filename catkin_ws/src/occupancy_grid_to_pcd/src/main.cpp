#include "OccupancyGridMap.h"
#include <Eigen/Dense>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <thread>

int main(int argc, char **argv) {
  ros::init(argc, argv, "occupancy_grid_to_pcd");

  if (argc != 6) {
    PCL_ERROR("Usage: %s <ogp_file1> <ogp_file2> <pcd_file1.pcd> <pcd_file2.pcd> leaf_size\n", argv[0]);
    return -1;
  }

  std::string ogp_file1 = argv[1];
  std::string ogp_file2 = argv[2];
  std::string pcd_file1 = argv[3];
  std::string pcd_file2 = argv[4];

  OccupancyGridMap ogm;

  nav_msgs::OccupancyGrid occupancy_grid1;
  nav_msgs::OccupancyGrid occupancy_grid2;
  ogm.loadOccupancyGridMap(ogp_file1, occupancy_grid1);
  ogm.loadOccupancyGridMap(ogp_file2, occupancy_grid2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>());

  ogm.occupancyGridToPCD(occupancy_grid1, cloud1);
  ogm.occupancyGridToPCD(occupancy_grid2, cloud2);

  pcl::io::savePCDFileASCII(pcd_file1, *cloud1);
  pcl::io::savePCDFileASCII(pcd_file2, *cloud2);

  std::cout << "Saved " << cloud1->points.size() << " data points to " << pcd_file1 << std::endl;
  std::cout << "Saved " << cloud2->points.size() << " data points to " << pcd_file2 << std::endl;

  Eigen::Matrix4f transformation_matrix;

  Eigen::Quaternionf quaternion(0.999805, -0.0197288, 0, 0);
  Eigen::Matrix3f rotation_matrix = quaternion.toRotationMatrix();
  std::cout << rotation_matrix << std::endl;
  Eigen::Vector3f translation_vector(-56.4116, -16.5114, 0.0);
  Eigen::Matrix4f transformation_matrix1;
  transformation_matrix1.setIdentity();                         // 初始化为单位矩阵
  transformation_matrix1.block<3, 3>(0, 0) = rotation_matrix;   // 设置旋转部分
  transformation_matrix1.block<3, 1>(0, 3) = translation_vector;// 设置平移部分
  float leaf_size = std::stof(argv[5]);// Example leaf size for downsampling
  ogm.performICP(cloud1, cloud2, transformation_matrix, leaf_size);

  // 应用变换矩阵到第一个点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud1(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud1, *transformed_cloud1, transformation_matrix1);

  // 可视化
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> original_source_cloud_color(cloud1, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud1, original_source_cloud_color, "original source cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_source_cloud_color(transformed_cloud1, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud1, transformed_source_cloud_color, "transformed source cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color(cloud2, 0, 0, 255);
  viewer->addPointCloud<pcl::PointXYZ>(cloud2, target_cloud_color, "target cloud");

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original source cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed source cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  std::ofstream outfile("transformation_matrix.txt");
  if (outfile.is_open()) {
    outfile << transformation_matrix << std::endl;
    outfile.close();
    std::cout << "Transformation matrix saved to transformation_matrix.txt" << std::endl;
  } else {
    std::cerr << "Unable to open file to save transformation matrix." << std::endl;
  }

  return 0;
}
