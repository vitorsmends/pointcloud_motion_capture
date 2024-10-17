#include "motion_capture.hpp"

MotionCapture::MotionCapture()
: Node("point_cloud_processor"),
  previous_position_(Eigen::Vector4f::Zero()),
  previous_velocity_(Eigen::Vector3f::Zero()),
  previous_time_(this->now()), previous_velocity_time_(this->now()),
  position_samples_(5), velocity_samples_(5), publish_rate_(10.0)
{
  subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/point_cloud/filtered", 10,
    std::bind(&MotionCapture::pointCloudCallback, this, std::placeholders::_1));
  position_publisher_ = this->create_publisher<geometry_msgs::msg::Point>(
    "object_position", 10);
  velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
    "object_velocity", 10);
  acceleration_publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>(
    "object_acceleration", 10);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)),
    std::bind(&MotionCapture::publishData, this));

  RCLCPP_INFO(this->get_logger(), "MotionCapture node initialized.");
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
  position_msg.x = centroid[0];
  position_msg.y = centroid[1];
  position_msg.z = centroid[2];

  position_publisher_->publish(position_msg);
}

Eigen::Vector3f MotionCapture::publishVelocity(const Eigen::Vector4f & current_position)
{
  rclcpp::Time current_time = this->now();
  double time_difference = (current_time - previous_time_).seconds();
  Eigen::Vector3f velocity = Eigen::Vector3f::Zero();

  if (time_difference > 0) {
    velocity[0] = (current_position[0] - previous_position_[0]) / time_difference;
    velocity[1] = (current_position[1] - previous_position_[1]) / time_difference;
    velocity[2] = (current_position[2] - previous_position_[2]) / time_difference;

    geometry_msgs::msg::Vector3 velocity_msg;
    velocity_msg.x = velocity[0];
    velocity_msg.y = velocity[1];
    velocity_msg.z = velocity[2];

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
    acceleration[0] = (current_velocity[0] - previous_velocity_[0]) / time_difference;
    acceleration[1] = (current_velocity[1] - previous_velocity_[1]) / time_difference;
    acceleration[2] = (current_velocity[2] - previous_velocity_[2]) / time_difference;

    geometry_msgs::msg::Vector3 acceleration_msg;
    acceleration_msg.x = acceleration[0];
    acceleration_msg.y = acceleration[1];
    acceleration_msg.z = acceleration[2];

    acceleration_publisher_->publish(acceleration_msg);
  }

  previous_velocity_ = current_velocity;
  previous_velocity_time_ = current_time;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotionCapture>());
  rclcpp::shutdown();
  return 0;
}
