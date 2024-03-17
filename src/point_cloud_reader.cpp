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
    PointCloudProcessor() : Node("point_cloud_processor") {
        load_parameters();

        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud/filtered", 10);
    }

private:
    void load_parameters() {
        std::string yaml_file_path = ament_index_cpp::get_package_share_directory("pointcloud_motion_capture") + "/config/colors.yaml";
        std::cout << yaml_file_path << std::endl;

        YAML::Node config = YAML::LoadFile(yaml_file_path);

        red_threshold_ = config["red_threshold"].as<int>();
        green_threshold_ = config["green_threshold"].as<int>();
        blue_threshold_ = config["blue_threshold"].as<int>();
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (const auto& point : *pcl_cloud) {
            if (point.r > red_threshold_ && point.g < green_threshold_ && point.b < blue_threshold_) {
                filtered_cloud->push_back(point);
            }
        }

        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header.frame_id = "camera_link";
        filtered_msg.header.stamp = this->get_clock()->now();

        publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    int red_threshold_;
    int green_threshold_;
    int blue_threshold_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
