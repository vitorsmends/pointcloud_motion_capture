// Copyright 2024 João Vitor Silva Mendes

#include "motion_capture.hpp"

namespace pointcloud_motion_capture
{
MotionCapture::MotionCapture()
: Node("motion_capture"),
  previous_position_(Eigen::Vector4f::Zero()),
  previous_velocity_(Eigen::Vector3f::Zero()),
  previous_time_(this->now()), previous_velocity_time_(this->now())
{
  loadParameters();

  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    point_cloud_topic_filtered_, queue_size_,
    std::bind(&MotionCapture::pointCloudCallback, this, std::placeholders::_1));

  position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
    position_publish_topic_, queue_size_);

  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
    velocity_publish_topic_, queue_size_);

  acceleration_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
    acceleration_publish_topic_, queue_size_);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(kMillisecondsPerSecond / publish_rate_)),
    std::bind(&MotionCapture::publishData, this));

  RCLCPP_INFO(this->get_logger(), "MotionCapture node initialized.");
}

void MotionCapture::loadParameters()
{
  this->declare_parameter<std::string>(
    "point_cloud_topic_filtered",
    kDefaultPointCloudTopicFiltered);
  this->declare_parameter<std::string>("position_publish_topic", kDefaultPositionPublishTopic);
  this->declare_parameter<std::string>("velocity_publish_topic", kDefaultVelocityPublishTopic);
  this->declare_parameter<std::string>(
    "acceleration_publish_topic",
    kDefaultAccelerationPublishTopic);
  this->declare_parameter<int>("position_samples", kDefaultPositionSamples);
  this->declare_parameter<int>("velocity_samples", kDefaultVelocitySamples);
  this->declare_parameter<double>("publish_rate", kDefaultPublishRate);
  this->declare_parameter<int>("queue_size", kDefaultQueueSize);
  point_cloud_topic_filtered_ = this->get_parameter("point_cloud_topic_filtered").as_string();
  position_publish_topic_ = this->get_parameter("position_publish_topic").as_string();
  velocity_publish_topic_ = this->get_parameter("velocity_publish_topic").as_string();
  acceleration_publish_topic_ = this->get_parameter("acceleration_publish_topic").as_string();
  position_samples_ = this->get_parameter("position_samples").as_int();
  velocity_samples_ = this->get_parameter("velocity_samples").as_int();
  publish_rate_ = this->get_parameter("publish_rate").as_double();
  queue_size_ = this->get_parameter("queue_size").as_int();
}

void MotionCapture::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*msg, *cloud);

  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*cloud, centroid);

  updatePositionSamples(centroid);
}

void MotionCapture::updatePositionSamples(const Eigen::Vector4f & centroid)
{
  if (static_cast<int>(position_queue_.size()) >= position_samples_) {
    position_queue_.pop();
  }
  position_queue_.push(centroid);
}

Eigen::Vector4f MotionCapture::getFilteredPosition()
{
  Eigen::Vector4f sum = Eigen::Vector4f::Zero();
  std::queue<Eigen::Vector4f> temp_queue = position_queue_;
  while (!temp_queue.empty()) {
    sum += temp_queue.front();
    temp_queue.pop();
  }
  return sum / position_queue_.size();
}

void MotionCapture::updateVelocitySamples(const Eigen::Vector3f & velocity)
{
  if (static_cast<int>(velocity_queue_.size()) >= velocity_samples_) {
    velocity_queue_.pop();
  }
  velocity_queue_.push(velocity);
}

Eigen::Vector3f MotionCapture::getFilteredVelocity()
{
  Eigen::Vector3f sum = Eigen::Vector3f::Zero();
  std::queue<Eigen::Vector3f> temp_queue = velocity_queue_;
  while (!temp_queue.empty()) {
    sum += temp_queue.front();
    temp_queue.pop();
  }
  return sum / velocity_queue_.size();
}

void MotionCapture::publishData()
{
  if (!position_queue_.empty()) {
    Eigen::Vector4f filtered_position = getFilteredPosition();
    publishPosition(filtered_position);

    Eigen::Vector3f current_velocity = publishVelocity(filtered_position);
    updateVelocitySamples(current_velocity);

    if (!velocity_queue_.empty()) {
      Eigen::Vector3f filtered_velocity = getFilteredVelocity();
      publishAcceleration(filtered_velocity);
    }

    previous_position_ = filtered_position;
    previous_time_ = this->now();
  }
}

void MotionCapture::publishPosition(const Eigen::Vector4f & centroid)
{
  geometry_msgs::msg::Point position_msg;
  position_msg.x = centroid[kCentroidX];
  position_msg.y = centroid[kCentroidY];
  position_msg.z = centroid[kCentroidZ];

  position_publisher_->publish(position_msg);
}

Eigen::Vector3f MotionCapture::publishVelocity(const Eigen::Vector4f & current_position)
{
  rclcpp::Time current_time = this->now();
  double time_difference = (current_time - previous_time_).seconds();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

  if (time_difference > 0) {
    velocity[kCentroidX] = (current_position[kCentroidX] - previous_position_[kCentroidX]) /
      time_difference;
    velocity[kCentroidY] = (current_position[kCentroidY] - previous_position_[kCentroidY]) /
      time_difference;
    velocity[kCentroidZ] = (current_position[kCentroidZ] - previous_position_[kCentroidZ]) /
      time_difference;

    geometry_msgs::msg::Vector3 velocity_msg;
    velocity_msg.x = velocity[kCentroidX];
    velocity_msg.y = velocity[kCentroidY];
    velocity_msg.z = velocity[kCentroidZ];

    velocity_publisher_->publish(velocity_msg);
  }

  return velocity;
}

void MotionCapture::publishAcceleration(const Eigen::Vector3f & current_velocity)
{
  rclcpp::Time current_time = this->now();
  double time_difference = (current_time - previous_velocity_time_).seconds();
  Eigen::Vector3f acceleration = Eigen::Vector3f::Zero();

  if (time_difference > 0) {
    acceleration[kVectorX] = (current_velocity[kVectorX] - previous_velocity_[kVectorX]) /
      time_difference;
    acceleration[kVectorY] = (current_velocity[kVectorY] - previous_velocity_[kVectorY]) /
      time_difference;
    acceleration[kVectorZ] = (current_velocity[kVectorZ] - previous_velocity_[kVectorZ]) /
      time_difference;

    geometry_msgs::msg::Vector3 acceleration_msg;
    acceleration_msg.x = acceleration[kVectorX];
    acceleration_msg.y = acceleration[kVectorY];
    acceleration_msg.z = acceleration[kVectorZ];

    acceleration_publisher_->publish(acceleration_msg);
  }

  previous_velocity_ = current_velocity;
  previous_velocity_time_ = current_time;
}

}  // namespace pointcloud_motion_capture

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pointcloud_motion_capture::MotionCapture>());
  rclcpp::shutdown();
  return 0;
}
