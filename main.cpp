#include <Eigen/Dense>
#include <chrono>
#include <fstream>
#include <iostream>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

void saveMatrixToFile(const Eigen::Matrix4f &matrix, const std::string &filename) {
  std::ofstream file(filename);
  if (file.is_open()) {
    file << matrix << std::endl;
    file.close();
    std::cout << "Rotation matrix saved to " << filename << std::endl;
  } else {
    std::cerr << "Unable to open file " << filename << " for writing." << std::endl;
  }
}

int main(int argc, char **argv) {
  if (argc != 4) {
    PCL_ERROR("Usage: %s source.pcd target.pcd leaf_size\n", argv[0]);
    return -1;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[1], *cloud_source) == -1) {
    PCL_ERROR("Couldn't read source file %s\n", argv[1]);
    return -1;
  }

  if (pcl::io::loadPCDFile<pcl::PointXYZ>(argv[2], *cloud_target) == -1) {
    PCL_ERROR("Couldn't read target file %s\n", argv[2]);
    return -1;
  }

  // Parse the leaf size parameter
  float leaf_size = std::stof(argv[3]);

  // Downsample the point clouds using VoxelGrid filter
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target_filtered(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setLeafSize(leaf_size, leaf_size, leaf_size);

  sor.setInputCloud(cloud_source);
  sor.filter(*cloud_source_filtered);
  std::cout << "Source cloud before filtering: " << cloud_source->size() << " points." << std::endl;
  std::cout << "Source cloud after filtering: " << cloud_source_filtered->size() << " points." << std::endl;

  sor.setInputCloud(cloud_target);
  sor.filter(*cloud_target_filtered);
  std::cout << "Target cloud before filtering: " << cloud_target->size() << " points." << std::endl;
  std::cout << "Target cloud after filtering: " << cloud_target_filtered->size() << " points." << std::endl;

  // Apply Statistical Outlier Removal to remove noise
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor_stat;
  sor_stat.setMeanK(50);
  sor_stat.setStddevMulThresh(1.0);

  sor_stat.setInputCloud(cloud_source_filtered);
  sor_stat.filter(*cloud_source_filtered);

  sor_stat.setInputCloud(cloud_target_filtered);
  sor_stat.filter(*cloud_target_filtered);

  // Initial alignment using RANSAC
  pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
  sac_ia.setMinSampleDistance(leaf_size);
  sac_ia.setMaxCorrespondenceDistance(5 * leaf_size);
  sac_ia.setMaximumIterations(1000);

  pcl::PointCloud<pcl::FPFHSignature33>::Ptr source_features(new pcl::PointCloud<pcl::FPFHSignature33>);
  pcl::PointCloud<pcl::FPFHSignature33>::Ptr target_features(new pcl::PointCloud<pcl::FPFHSignature33>);

  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setRadiusSearch(leaf_size * 5);

  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setRadiusSearch(leaf_size * 2);

  pcl::PointCloud<pcl::Normal>::Ptr source_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::Normal>::Ptr target_normals(new pcl::PointCloud<pcl::Normal>);

  ne.setInputCloud(cloud_source_filtered);
  ne.compute(*source_normals);
  fpfh.setInputCloud(cloud_source_filtered);
  fpfh.setInputNormals(source_normals);
  fpfh.compute(*source_features);

  ne.setInputCloud(cloud_target_filtered);
  ne.compute(*target_normals);
  fpfh.setInputCloud(cloud_target_filtered);
  fpfh.setInputNormals(target_normals);
  fpfh.compute(*target_features);

  sac_ia.setInputSource(cloud_source_filtered);
  sac_ia.setSourceFeatures(source_features);
  sac_ia.setInputTarget(cloud_target_filtered);
  sac_ia.setTargetFeatures(target_features);

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_source(new pcl::PointCloud<pcl::PointXYZ>);
  sac_ia.align(*aligned_source);

  if (sac_ia.hasConverged()) {
    std::cout << "RANSAC alignment has converged, score is " << sac_ia.getFitnessScore() << std::endl;
  } else {
    std::cout << "RANSAC alignment has not converged." << std::endl;
  }

  Eigen::Matrix4f initial_alignment = sac_ia.getFinalTransformation();
  pcl::transformPointCloud(*cloud_source_filtered, *aligned_source, initial_alignment);

  // ICP refinement
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setMaximumIterations(1000);
  icp.setTransformationEpsilon(1e-9);
  icp.setEuclideanFitnessEpsilon(1);
  icp.setMaxCorrespondenceDistance(leaf_size * 5);

  icp.setInputSource(aligned_source);
  icp.setInputTarget(cloud_target_filtered);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  if (icp.hasConverged()) {
    std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
  } else {
    std::cout << "ICP has not converged." << std::endl;
  }

  Eigen::Matrix4f transformation = icp.getFinalTransformation() * initial_alignment;
  std::cout << "Final transformation matrix:" << std::endl
            << transformation << std::endl;

  // Save the rotation matrix to a file
  saveMatrixToFile(transformation, "transformation_matrix.txt");

  pcl::io::savePCDFileASCII("output.pcd", Final);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_source_filtered1(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::transformPointCloud(*cloud_source_filtered, *transformed_cloud_source_filtered1, transformation);

  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);

  // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_color(cloud_source_filtered, 0, 255, 0);
  // viewer->addPointCloud<pcl::PointXYZ>(cloud_source_filtered, source_color, "source cloud");
  // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(cloud_target_filtered, 255, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud_target_filtered, target_color, "target cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_color(transformed_cloud_source_filtered1, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ>(transformed_cloud_source_filtered1, final_color, "transformed_cloud_source_filtered1");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "transformed_cloud_source_filtered1");

  viewer->addCoordinateSystem(1.0);
  viewer->initCameraParameters();

  while (!viewer->wasStopped()) {
    viewer->spinOnce(100);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}
