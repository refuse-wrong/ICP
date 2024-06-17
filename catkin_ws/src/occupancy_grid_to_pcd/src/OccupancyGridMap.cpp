#include "OccupancyGridMap.h"
#include <ros/serialization.h>
#include <pcl/common/transforms.h>

void OccupancyGridMap::loadOccupancyGridMap(const std::string &file_name, nav_msgs::OccupancyGrid &occupancy_grid) {
    std::string file_path = file_name;  // 构建完整的文件路径
    std::ifstream file(file_path, std::ios::in | std::ios::binary);
    if (!file.is_open()) {
        ROS_ERROR("Failed to open file: %s", file_path.c_str());
        return;
    }

    ROS_INFO("OccupancyGrid message load from file.");

    file.seekg(0, std::ios::end);
    size_t file_size = file.tellg();
    file.seekg(0, std::ios::beg);

    boost::shared_array<uint8_t> buffer(new uint8_t[file_size]);
    file.read(reinterpret_cast<char *>(buffer.get()), file_size);
    file.close();

    // 反序列化消息
    ros::serialization::IStream stream(buffer.get(), file_size);
    ros::serialization::deserialize(stream, occupancy_grid);
}

void OccupancyGridMap::occupancyGridToPCD(const nav_msgs::OccupancyGrid &occupancyGrid, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {
    float resolution = occupancyGrid.info.resolution;
    float origin_x = occupancyGrid.info.origin.position.x;
    float origin_y = occupancyGrid.info.origin.position.y;

    for (int i = 0; i < occupancyGrid.info.width; ++i) {
        for (int j = 0; j < occupancyGrid.info.height; ++j) {
            int index = j * occupancyGrid.info.width + i;
            int occupancy_status = occupancyGrid.data[index];

            if (occupancy_status == 100) { // Occupied
                pcl::PointXYZ point;
                point.x = origin_x + (i + 0.5) * resolution;
                point.y = origin_y + (j + 0.5) * resolution;
                point.z = 0.0;
                cloud->points.push_back(point);
            }
        }
    }

    cloud->width = cloud->points.size();
    cloud->height = 1;
    cloud->is_dense = true;
}

void OccupancyGridMap::performICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source_cloud,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &target_cloud,
                                  Eigen::Matrix4f &transformation_matrix,
                                  float leaf_size) {
  // 预处理：去噪声
  pcl::PointCloud<pcl::PointXYZ>::Ptr source_filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_filtered(new pcl::PointCloud<pcl::PointXYZ>());

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(source_cloud);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*source_filtered);

  sor.setInputCloud(target_cloud);
  sor.filter(*target_filtered);

  // 预处理：下采样
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
  voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);

  voxel_filter.setInputCloud(source_filtered);
  voxel_filter.filter(*source_filtered);
  std::cout << "Filtered source cloud: " << source_filtered->points.size() << " points" << std::endl;

  voxel_filter.setInputCloud(target_filtered);
  voxel_filter.filter(*target_filtered);
  std::cout << "Filtered target cloud: " << target_filtered->points.size() << " points" << std::endl;

  // 设置ICP参数
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
//   icp.setMaxCorrespondenceDistance(5);// 设置对应点对的最大距离
//   icp.setMaximumIterations(50);          // 设置最大迭代次数
//   icp.setTransformationEpsilon(1e-8);    // 设置转化容差
//   icp.setEuclideanFitnessEpsilon(1);     // 设置欧几里得适应度容差

  icp.setInputSource(source_filtered);
  icp.setInputTarget(target_filtered);

  pcl::PointCloud<pcl::PointXYZ> final_cloud;
  icp.align(final_cloud);

  if (icp.hasConverged()) {
    transformation_matrix = icp.getFinalTransformation();
    std::cout << "ICP converged with score: " << icp.getFitnessScore() << std::endl;
    std::cout << "Transformation matrix: \n"
              << transformation_matrix << std::endl;
  } else {
    std::cout << "ICP did not converge." << std::endl;
  }
}
