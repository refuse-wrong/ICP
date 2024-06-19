#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <cmath> // for M_PI

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
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0] << " <transformation_matrix_file1> <transformation_matrix_file2>" << std::endl;
    return -1;
  }

  std::string file_path1 = argv[1];
  std::string file_path2 = argv[2];

  Eigen::Matrix4d matrix1 = readTransformationMatrix(file_path1);
  Eigen::Matrix4d matrix2 = readTransformationMatrix(file_path2);

  Eigen::Vector3d translation_error = computeTranslationError(matrix1, matrix2);
  double yaw_error = computeYawError(matrix1, matrix2);

  double total_translation_error = translation_error.norm();

  std::cout << "Translation Error: x: " << translation_error.x() << "m"
            << ", y: " << translation_error.y() << "m"
            << ", z: " << translation_error.z() << "m"
            << ", total: " << total_translation_error << "m" << std::endl;

  std::cout << "Yaw Error: " << yaw_error << "°" << std::endl;

  return 0;
}
