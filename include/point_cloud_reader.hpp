#ifndef POINT_CLOUD_READER_HPP
#define POINT_CLOUD_READER_HPP

#include <iostream>
#include <memory>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace pointcloud_motion_capture
{
class PointCloudProcessor : public rclcpp::Node {
 public:
    PointCloudProcessor();

 private:
    const int kRedThresholdDefault {40};
    const int kGreenThresholdDefault {30};
    const int kBlueThresholdDefault {30};
    const float kUpdateRate {10.0};
    const std::string kPointCloudTopicRaw {"/camera/camera/depth/color/points"};
    const std::string kPointCloudTopicFiltered {"/point_cloud/filtered"};

    void load_parameters();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    int red_threshold_ {};
    int green_threshold_ {};
    int blue_threshold_ {};
    std::string frame_id_ {"camera_link"};
    std::string point_cloud_topic_raw_ {};
    std::string point_cloud_topic_filtered_ {};
    float update_rate_ {};
    sensor_msgs::msg::PointCloud2 filtered_msg_;
};

}  // namespace pointcloud_motion_capture

#endif  // POINT_CLOUD_READER_HPP
