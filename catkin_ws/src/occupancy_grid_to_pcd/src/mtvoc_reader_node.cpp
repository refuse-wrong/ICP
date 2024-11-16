#include <Eigen/Geometry>
#include <fstream>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>

Eigen::Matrix3d QuaternionToRotationMatrix(const geometry_msgs::Quaternion &q) {
  Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();
  return rotation_matrix;
}

// 创建 4x4 变换矩阵
Eigen::Matrix4d PoseToTransformationMatrix(const geometry_msgs::Pose &pose) {
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  Eigen::Matrix3d rotation_matrix = quaternion.toRotationMatrix();

  Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();
  transformation_matrix.block<3, 3>(0, 0) = rotation_matrix;
  transformation_matrix(0, 3) = pose.position.x;
  transformation_matrix(1, 3) = pose.position.y;
  transformation_matrix(2, 3) = pose.position.z;

  return transformation_matrix;
}

std::vector<Eigen::Matrix4d> ReadMTvoc2D(const std::string &input_file_path, const std::string &output_file_path) {
  std::vector<Eigen::Matrix4d> transformation_matrices;
  std::ifstream in_file(input_file_path, std::ios::in | std::ios::binary);
  if (!in_file) {
    std::cerr << "can't open the file: " << input_file_path << std::endl;
    return transformation_matrices;
  }

  // 读取文件中的数据
  std::vector<std::vector<geometry_msgs::Pose>> MTvoc2D_read;
  while (true) {
    std::vector<geometry_msgs::Pose> poses;
    while (true) {
      geometry_msgs::Pose pose;
      // 读取位置向量
      if (!in_file.read(reinterpret_cast<char *>(&pose.position), sizeof(pose.position))) {
        break;// 如果读取失败或到达文件末尾，退出内层循环
      }
      // 读取四元数
      if (!in_file.read(reinterpret_cast<char *>(&pose.orientation), sizeof(pose.orientation))) {
        break;// 如果读取失败或到达文件末尾，退出内层循环
      }
      poses.push_back(pose);
    }
    if (poses.empty()) {
      break;// 如果当前 vector 为空，则退出外层循环
    }
    MTvoc2D_read.push_back(poses);
  }

  in_file.close();

  // 将读取的数据保存到txt文件中
  std::ofstream out_file(output_file_path);
  if (!out_file) {
    std::cerr << "can't open the file: " << output_file_path << std::endl;
    return transformation_matrices;
  }

  for (const auto &poses : MTvoc2D_read) {
    for (const auto &pose : poses) {
      // 创建 4x4 变换矩阵
      Eigen::Matrix4d transformation_matrix = PoseToTransformationMatrix(pose);
      if (!transformation_matrix.isIdentity()) {
        transformation_matrices.push_back(transformation_matrix);
        out_file << transformation_matrix << "\n\n";
      }
    }
    out_file << std::endl;// 在每组姿势之间加一个空行
  }

  out_file.close();
  return transformation_matrices;
}

// 读取变换矩阵
Eigen::Matrix4d readTransformationMatrix(const std::string &file_path) {
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  std::ifstream file(file_path);
  if (file.is_open()) {
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        file >> matrix(i, j);
      }
    }
    file.close();
  } else {
    std::cerr << "Unable to open file: " << file_path << std::endl;
  }
  return matrix;
}

// 计算平移误差
Eigen::Vector3d computeTranslationError(const Eigen::Matrix4d &matrix1, const Eigen::Matrix4d &matrix2) {
  Eigen::Vector3d translation1 = matrix1.block<3, 1>(0, 3);
  Eigen::Vector3d translation2 = matrix2.block<3, 1>(0, 3);
  return translation1 - translation2;
}

// 计算旋转误差
double computeYawError(const Eigen::Matrix4d &matrix1, const Eigen::Matrix4d &matrix2) {
  Eigen::Matrix3d rotation1 = matrix1.block<3, 3>(0, 0);
  Eigen::Matrix3d rotation2 = matrix2.block<3, 3>(0, 0);
  // 计算相对旋转矩阵
  Eigen::Matrix3d rotation_error = rotation1.transpose() * rotation2;
  // 将相对旋转矩阵转换为轴角表示
  Eigen::AngleAxisd angle_axis(rotation_error);
  // 返回旋转角度（弧度）
  return angle_axis.angle() * 180.0 / M_PI;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mtvoc_reader");
  ros::NodeHandle nh;

  if (argc < 4) {
    std::cerr << "Usage: mtvoc_reader_node <input_file_path> <output_file_path> <gt_file_path>" << std::endl;
    return 1;
  }

  std::string input_file_path = argv[1]; // binary file
  std::string output_file_path = argv[2]; // txt file
  std::string gt_file_path = argv[3]; // txt file

  Eigen::Matrix4d gt_matrix = readTransformationMatrix(gt_file_path);
  std::vector<Eigen::Matrix4d> transformation_matrices = ReadMTvoc2D(input_file_path, output_file_path);
  
  for (size_t i = 0; i < transformation_matrices.size(); ++i) {
    Eigen::Matrix4d transformation_matrix = transformation_matrices[i];
    Eigen::Vector3d translation_error = computeTranslationError(transformation_matrix, gt_matrix);
    double yaw_error = computeYawError(transformation_matrix, gt_matrix);
    double total_translation_error = translation_error.norm();

    std::cout << "Translation Error: x: " << translation_error.x() << "m"
              << ", y: " << translation_error.y() << "m"
              << ", z: " << translation_error.z() << "m"
              << ", total: " << total_translation_error << "m" << std::endl;

    std::cout << "Yaw Error: " << yaw_error << "°" << std::endl;
  }


  return 0;
}
