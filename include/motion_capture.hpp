// Copyright 2024 João Vitor Silva Mendes

#ifndef MOTION_CAPTURE_HPP_
#define MOTION_CAPTURE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <queue>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

class MotionCapture : public rclcpp::Node
{
public:
  MotionCapture();

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void updatePositionSamples(const Eigen::Vector4f & centroid);
  Eigen::Vector4f getFilteredPosition();
  void updateVelocitySamples(const Eigen::Vector3f & velocity);
  Eigen::Vector3f getFilteredVelocity();
  void publishData();
  void publishPosition(const Eigen::Vector4f & centroid);
  Eigen::Vector3f publishVelocity(const Eigen::Vector4f & current_position);
  void publishAcceleration(const Eigen::Vector3f & current_velocity);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acceleration_publisher_;
  std::queue<Eigen::Vector4f> position_queue_;
  std::queue<Eigen::Vector3f> velocity_queue_;
  Eigen::Vector4f previous_position_;
  Eigen::Vector3f previous_velocity_;
  rclcpp::Time previous_time_;
  rclcpp::Time previous_velocity_time_;
  int position_samples_;
  int velocity_samples_;
  double publish_rate_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // MOTION_CAPTURE_HPP_
