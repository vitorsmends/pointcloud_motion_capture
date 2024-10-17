#include "point_cloud_reader.hpp"

namespace pointcloud_motion_capture
{
PointCloudProcessor::PointCloudProcessor() : Node("point_cloud_processor") {
    load_parameters();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_raw_, update_rate_,
        std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        point_cloud_topic_filtered_, update_rate_);

    RCLCPP_INFO(this->get_logger(), "PointCloudProcessor node initialized.");
}

void PointCloudProcessor::load_parameters() {
    this->declare_parameter<int>("red_threshold", kRedThresholdDefault);
    this->declare_parameter<int>("green_threshold", kGreenThresholdDefault);
    this->declare_parameter<int>("blue_threshold", kBlueThresholdDefault);
    this->declare_parameter<std::string>("point_cloud_topic_raw", kPointCloudTopicRaw);
    this->declare_parameter<std::string>("point_cloud_topic_filtered", kPointCloudTopicFiltered);
    this->declare_parameter<float>("update_rate", kUpdateRate);

    this->get_parameter("red_threshold", red_threshold_);
    this->get_parameter("green_threshold", green_threshold_);
    this->get_parameter("blue_threshold", blue_threshold_);
    this->get_parameter("point_cloud_topic_raw", point_cloud_topic_raw_);
    this->get_parameter("point_cloud_topic_filtered", point_cloud_topic_filtered_);
    this->get_parameter("update_rate", update_rate_);

    RCLCPP_INFO(this->get_logger(), "Loading Parameters...");
    RCLCPP_INFO(this->get_logger(), "Filter parameters:");
    RCLCPP_INFO(this->get_logger(), "   Red threshold: %d", red_threshold_);
    RCLCPP_INFO(this->get_logger(), "   Green threshold: %d", green_threshold_);
    RCLCPP_INFO(this->get_logger(), "   Blue threshold: %d", blue_threshold_);
    RCLCPP_INFO(this->get_logger(), "Reading point cloud from: %s", point_cloud_topic_raw_.c_str());
    RCLCPP_INFO(this->get_logger(), "Publishing filtered point cloud on: %s", point_cloud_topic_filtered_.c_str());
    RCLCPP_INFO(this->get_logger(), "Update Rate: %f Hz", update_rate_);
}


void PointCloudProcessor::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *pcl_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (const auto& point : *pcl_cloud) {
        if (point.r > red_threshold_ && point.g < green_threshold_ && point.b < blue_threshold_) {
            filtered_cloud->push_back(point);
        }
    }

    pcl::toROSMsg(*filtered_cloud, filtered_msg_);
    filtered_msg_.header.frame_id = frame_id_;
    filtered_msg_.header.stamp = this->get_clock()->now();

    publisher_->publish(filtered_msg_);
}

} // namespace pointcloud_motion_capture

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<pointcloud_motion_capture::PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
