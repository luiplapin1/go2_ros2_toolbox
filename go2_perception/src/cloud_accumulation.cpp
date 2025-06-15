#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "tf2_sensor_msgs/tf2_sensor_msgs.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2/LinearMath/Transform.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

sensor_msgs::msg::PointCloud2 accumulated_cloud;
geometry_msgs::msg::PoseStamped rbt_pose;
std::vector<sensor_msgs::msg::PointCloud2::ConstSharedPtr> clouds;

void cloudCallback(
sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
    // 创建一个位姿变换
    double quatx= rbt_pose.pose.orientation.x;
    double quaty= rbt_pose.pose.orientation.y;
    double quatz= rbt_pose.pose.orientation.z;
    double quatw= rbt_pose.pose.orientation.w;

    tf2::Quaternion qq(quatx, quaty, quatz, quatw);
    tf2::Matrix3x3 m(qq);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf2::Quaternion q;
    q.setRPY(0.0, 165.0 * M_PI / 180.0, yaw); // 设置旋转部分

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.transform.translation.x = rbt_pose.pose.position.x;
    transform_stamped.transform.translation.y = rbt_pose.pose.position.y;
    transform_stamped.transform.translation.z = 0.05;
    transform_stamped.transform.rotation.x = q.x();
    transform_stamped.transform.rotation.y = q.y();
    transform_stamped.transform.rotation.z = q.z();
    transform_stamped.transform.rotation.w = q.w();

    // 创建一个空的点云来存储转换后的数据
    auto transformed_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();

    // 使用 tf2::doTransform 函数来应用位姿变换
    tf2::doTransform(*cloud_msg, *transformed_cloud, transform_stamped);

    // 将新的点云添加到 clouds 中
    clouds.push_back(cloud_msg);

    // 如果 clouds 中的点云数量超过了一定的限制，删除最旧的点云
    if (clouds.size() > 30)
    {
        clouds.erase(clouds.begin());
    }

    // 创建一个新的点云来存储合并后的点云
    auto merged_cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
    *merged_cloud = *clouds[0];

    // 遍历 clouds 并将所有的点云合并到 merged_cloud 中
    for (size_t it = 1; it < clouds.size(); it++)
    {
        merged_cloud->width += clouds[it]->width;
        merged_cloud->row_step += clouds[it]->row_step;
        merged_cloud->data.insert(merged_cloud->data.end(), clouds[it]->data.begin(), clouds[it]->data.end());
    }

    // 过滤掉高度高于1.0或低于0.2的点
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    filtered_cloud.header = merged_cloud->header;
    filtered_cloud.height = 1;
    filtered_cloud.is_dense = false;
    filtered_cloud.is_bigendian = merged_cloud->is_bigendian;
    filtered_cloud.fields = merged_cloud->fields;
    filtered_cloud.point_step = merged_cloud->point_step;
    filtered_cloud.row_step = 0;

    // 遍历点云数据
    for (size_t i = 0; i < merged_cloud->width * merged_cloud->height; i++)
    {
        float x, y, z;
        memcpy(&x, &merged_cloud->data[i * merged_cloud->point_step + merged_cloud->fields[0].offset], sizeof(float));
        memcpy(&y, &merged_cloud->data[i * merged_cloud->point_step + merged_cloud->fields[1].offset], sizeof(float));
        memcpy(&z, &merged_cloud->data[i * merged_cloud->point_step + merged_cloud->fields[2].offset], sizeof(float));

        // 只保留高度在0.2到1.0之间的点
        if (z >= 0.2 && z <= 1.0) {
            filtered_cloud.data.insert(filtered_cloud.data.end(),
                                       &merged_cloud->data[i * merged_cloud->point_step],
                                       &merged_cloud->data[(i + 1) * merged_cloud->point_step]);
            filtered_cloud.row_step += merged_cloud->point_step;
            filtered_cloud.width++;
        }
    }

    accumulated_cloud = filtered_cloud;
    accumulated_cloud.header.frame_id = "odom";
}

void poseCallback(
  geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
{
  rbt_pose = *pose_msg;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::QoS qos = rclcpp::SensorDataQoS();
  qos.reliable();

  auto node = std::make_shared<rclcpp::Node>("cloud_accumulations");
  auto pub =
    node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", qos);

  auto sub2 = node->create_subscription<geometry_msgs::msg::PoseStamped>(
  "/utlidar/robot_pose", rclcpp::SensorDataQoS(),
  std::bind(&poseCallback, std::placeholders::_1));

  auto sub = node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/utlidar/cloud_deskewed", rclcpp::SensorDataQoS(),
    std::bind(&cloudCallback, std::placeholders::_1));

  sensor_msgs::msg::LaserScan laser_scan;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  rclcpp::Rate rate(50.0);
  while (rclcpp::ok()) {
    accumulated_cloud.header.stamp = node->get_clock()->now();
    pub->publish(accumulated_cloud);

    executor.spin_some();
    rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
