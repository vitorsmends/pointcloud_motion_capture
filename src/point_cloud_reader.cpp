#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class PointCloudProcessor : public rclcpp::Node {
public:
    PointCloudProcessor() : Node("point_cloud_processor") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/camera/depth/color/points", 10,
            std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/point_cloud/filtered", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Converter a mensagem ROS2 PointCloud2 para uma nuvem de pontos PCL
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *pcl_cloud);

        // Filtrar pontos próximos da cor vermelha
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        for (const auto& point : *pcl_cloud) {
            if (point.r > 40 && point.g < 30 && point.b < 30) {
                filtered_cloud->push_back(point);
            }
        }

        // Converter a nuvem de pontos filtrada PCL de volta para uma mensagem ROS2 PointCloud2
        sensor_msgs::msg::PointCloud2 filtered_msg;
        pcl::toROSMsg(*filtered_cloud, filtered_msg);
        filtered_msg.header.frame_id = "camera_link";
        filtered_msg.header.stamp = this->get_clock()->now();

        // Publicar a nuvem de pontos filtrada
        publisher_->publish(filtered_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
