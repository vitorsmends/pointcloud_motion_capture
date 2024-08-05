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

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor();

private:
    void load_parameters();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    int red_threshold_;
    int green_threshold_;
    int blue_threshold_;
};

#endif // POINT_CLOUD_READER_HPP
