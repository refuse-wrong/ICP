#include <chrono>
#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <sstream>
#include <thread>

// Function to load point cloud from a CSV file
bool loadPointCloudFromCSV(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  std::ifstream file(filename);
  if (!file.is_open()) {
    std::cerr << "Failed to open file " << filename << " for reading" << std::endl;
    return false;
  }

  std::string line;
  // Skip the header
  std::getline(file, line);

  while (std::getline(file, line)) {
    std::istringstream line_stream(line);
    std::string x_str, y_str, z_str;

    std::getline(line_stream, x_str, ',');
    std::getline(line_stream, y_str, ',');
    std::getline(line_stream, z_str, ',');

    pcl::PointXYZ point;
    point.x = std::stof(x_str);
    point.y = std::stof(y_str);
    point.z = std::stof(z_str);

    cloud->points.push_back(point);
  }

  cloud->width = cloud->points.size();
  cloud->height = 1;// Unorganized point cloud
  cloud->is_dense = false;

  file.close();
  std::cout << "Point cloud loaded from " << filename << std::endl;
  return true;
}

int main(int argc, char **argv) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <source_pcd_file> <target_csv_file>" << std::endl;
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

  // Load source PCD file
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_source) == -1) {
    PCL_ERROR("Couldn't read source file %s\n", argv[1]);
    return -1;
  }

  // Load target CSV file
  if (!loadPointCloudFromCSV(argv[2], cloud_target)) {
    PCL_ERROR("Couldn't read target file %s\n", argv[2]);
    return -1;
  }

  std::cout << "Source point cloud has " << cloud_source->points.size() << " points." << std::endl;
  std::cout << "Target point cloud has " << cloud_target->points.size() << " points." << std::endl;

  // Visualize the point clouds
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_source, source_color, "source cloud");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_target, target_color, "target cloud");

  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
