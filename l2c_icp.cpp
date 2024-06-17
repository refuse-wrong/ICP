#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
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

void saveMatrixToFile(const Eigen::Matrix4f &matrix, const std::string &filename) {
  std::ofstream file(filename);
  if (file.is_open()) {
    file << matrix << std::endl;
    file.close();
    std::cout << "Transformation matrix saved to " << filename << std::endl;
  } else {
    std::cerr << "Unable to open file " << filename << " for writing." << std::endl;
  }
}

// Function to perform ICP between two point clouds
Eigen::Matrix4f performICP(pcl::PointCloud<pcl::PointXYZ>::Ptr source,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr target,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr output,
                           float max_correspondence_distance) {
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaxCorrespondenceDistance(max_correspondence_distance);
  icp.setMaximumIterations(1000);
  icp.setTransformationEpsilon(1e-8);
  icp.setEuclideanFitnessEpsilon(1e-6);

  icp.setInputSource(source);
  icp.setInputTarget(target);

  icp.align(*output);

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  if (icp.hasConverged()) {
    std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix:" << std::endl
              << icp.getFinalTransformation() << std::endl;
    transformation = icp.getFinalTransformation();
  } else {
    std::cout << "ICP has not converged." << std::endl;
  }

  return transformation;
}

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void *user_data) {
  Eigen::Matrix4f *transformation = reinterpret_cast<Eigen::Matrix4f *>(user_data);
  float step = 0.01;
  if (event.getKeySym() == "Left" && event.keyDown()) {
    (*transformation)(0, 3) -= step;
  } else if (event.getKeySym() == "Right" && event.keyDown()) {
    (*transformation)(0, 3) += step;
  } else if (event.getKeySym() == "Up" && event.keyDown()) {
    (*transformation)(1, 3) += step;
  } else if (event.getKeySym() == "Down" && event.keyDown()) {
    (*transformation)(1, 3) -= step;
  } else if (event.getKeySym() == "PageUp" && event.keyDown()) {
    (*transformation)(2, 3) += step;
  } else if (event.getKeySym() == "PageDown" && event.keyDown()) {
    (*transformation)(2, 3) -= step;
  }
}

void updateTransformedCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr source, pcl::PointCloud<pcl::PointXYZ>::Ptr transformed, const Eigen::Matrix4f &transformation) {
  pcl::transformPointCloud(*source, *transformed, transformation);
}

int main(int argc, char **argv) {
  if (argc != 5) {
    PCL_ERROR("Usage: %s source.csv target.csv leaf_size height_limit\n", argv[0]);
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

  // Load source CSV file
  if (!loadPointCloudFromCSV(argv[1], cloud_source)) {
    PCL_ERROR("Couldn't read source file %s\n", argv[1]);
    return -1;
  }

  // Load target CSV file
  if (!loadPointCloudFromCSV(argv[2], cloud_target)) {
    PCL_ERROR("Couldn't read target file %s\n", argv[2]);
    return -1;
  }

  // Apply height restriction to the source cloud
  float height_limit = std::stof(argv[4]);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto &point : cloud_source->points) {
    if (point.z <= height_limit) {
      cloud_source_filtered->points.push_back(point);
    }
  }

  cloud_source_filtered->width = cloud_source_filtered->points.size();
  cloud_source_filtered->height = 1;// Unorganized point cloud
  cloud_source_filtered->is_dense = false;

  float leaf_size = std::stof(argv[3]);

  // Downsample the dense point cloud (source)
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
  voxel_grid.setInputCloud(cloud_source_filtered);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid.filter(*cloud_source_voxel_filtered);

  std::cout << "Source cloud size: " << cloud_source->size() << " points." << std::endl;
  std::cout << "Filtered source cloud size: " << cloud_source_filtered->size() << " points." << std::endl;
  std::cout << "Voxel filtered source cloud size: " << cloud_source_voxel_filtered->size() << " points." << std::endl;
  std::cout << "Target cloud size: " << cloud_target->size() << " points." << std::endl;

  // Perform ICP between sparse and dense point clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
  Eigen::Matrix4f transformation = performICP(cloud_source_voxel_filtered, cloud_target, cloud_final, leaf_size * 5);

  // Transform the original cloud_source based on the final transformation
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*cloud_source, *cloud_source_transformed, transformation);

  // Save the initial transformation matrix to a file
  saveMatrixToFile(transformation, "initial_transformation_matrix.txt");

  // Initialize PCLVisualizer
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

//   pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source, 0, 0, 255);
//   viewer->addPointCloud<pcl::PointXYZ>(cloud_source, source_color, "source cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_target, target_color, "target cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transformed_source_color(cloud_source_transformed, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_source_transformed, transformed_source_color, "transformed source cloud");

//   viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed source cloud");
  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  // Register keyboard callback for manual adjustment
  viewer->registerKeyboardCallback(keyboardEventOccurred, (void *) &transformation);

  while (!viewer->wasStopped()) {
    // Update the transformed source cloud based on the current transformation matrix
    updateTransformedCloud(cloud_source, cloud_source_transformed, transformation);
    viewer->updatePointCloud(cloud_source_transformed, transformed_source_color, "transformed source cloud");
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Save the manually adjusted transformation matrix to a file
  saveMatrixToFile(transformation, "manual_transformation_matrix.txt");

  return 0;
}
