#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// 计算给定 Pose 的逆变换
geometry_msgs::Pose invertPose(const geometry_msgs::Pose T) {
  geometry_msgs::Pose inverted_pose;

  // 提取平移向量 t
  double tx = T.position.x;
  double ty = T.position.y;
  double tz = T.position.z;

  // 提取四元数
  tf::Quaternion q(T.orientation.x, T.orientation.y, T.orientation.z, T.orientation.w);
  tf::Matrix3x3 R(q);

  // 计算 R 的转置 Rt (因为 R 是正交矩阵, R 的转置就是 R 的逆)
  tf::Matrix3x3 Rt = R.transpose();

  // 计算逆平移向量 -Rt * t
  tf::Vector3 t(tx, ty, tz);
  tf::Vector3 t_inv = -(Rt * t);

  // 设置逆变换的平移部分
  inverted_pose.position.x = t_inv.x();
  inverted_pose.position.y = t_inv.y();
  inverted_pose.position.z = t_inv.z();

  // 设置逆变换的旋转部分，转置的旋转矩阵再转回四元数
  tf::Quaternion q_inv;
  Rt.getRotation(q_inv);
  inverted_pose.orientation.x = q_inv.x();
  inverted_pose.orientation.y = q_inv.y();
  inverted_pose.orientation.z = q_inv.z();
  inverted_pose.orientation.w = q_inv.w();

  return inverted_pose;
}

int main(int argc, char **argv) { 
  ros::init(argc, argv, "pose_inverter");
  ros::NodeHandle nh;

  // 创建两个发布器，一个用于原始Pose，一个用于逆变换Pose
  ros::Publisher original_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("original_pose", 10);
  ros::Publisher inverted_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("inverted_pose", 10);

  tf::Transform Transform;
  tf::TransformBroadcaster broadcaster;
  tf::Transform Transform1;
  tf::TransformBroadcaster broadcaster1;
  ros::Rate loop_rate(1);// 1 Hz

  while (ros::ok()) {
    // 定义一个原始的Pose
    geometry_msgs::Pose original_pose;
    original_pose.position.x = 1.0;
    original_pose.position.y = 2.0;
    original_pose.position.z = 3.0;
    original_pose.orientation.x = 0.0;
    original_pose.orientation.y = 0.7071;
    original_pose.orientation.z = 0;
    original_pose.orientation.w = 0.7071;

    // 计算其逆变换
    geometry_msgs::Pose inverted_pose = invertPose(original_pose);

    tf::poseMsgToTF(original_pose, Transform);
    broadcaster.sendTransform(tf::StampedTransform(Transform, ros::Time::now(), "world", "test1"));

    tf::poseMsgToTF(inverted_pose, Transform1);
    broadcaster1.sendTransform(tf::StampedTransform(Transform1, ros::Time::now(), "test2", "world"));

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
