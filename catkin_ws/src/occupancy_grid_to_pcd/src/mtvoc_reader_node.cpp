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

void ReadMTvoc2D(const std::string &input_file_path, const std::string &output_file_path) {
  std::ifstream in_file(input_file_path, std::ios::in | std::ios::binary);
  if (!in_file) {
    std::cerr << "can't open the file: " << input_file_path << std::endl;
    return;
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
    return;
  }

  for (const auto &poses : MTvoc2D_read) {
    for (const auto &pose : poses) {
      // 转换四元数到旋转矩阵
      Eigen::Matrix3d rotation_matrix = QuaternionToRotationMatrix(pose.orientation);

      out_file << "Position: "
               << pose.position.x << " "
               << pose.position.y << " "
               << pose.position.z << " "
               << "Orientation (Rotation Matrix):\n"
               << rotation_matrix << "\n\n";
    }
    out_file << std::endl;// 在每组姿势之间加一个空行
  }

  out_file.close();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mtvoc_reader");
  ros::NodeHandle nh;

  if (argc < 3) {
    std::cerr << "Usage: mtvoc_reader_node <input_file_path> <output_file_path>" << std::endl;
    return 1;
  }

  std::string input_file_path = argv[1];
  std::string output_file_path = argv[2];

  ReadMTvoc2D(input_file_path, output_file_path);

  return 0;
}
