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
private:
    /**
     * @brief Default value for the red color threshold.
     */
    const int kRedThresholdDefault {40};

    /**
     * @brief Default value for the green color threshold.
     */
    const int kGreenThresholdDefault {30};

    /**
     * @brief Default value for the blue color threshold.
     */
    const int kBlueThresholdDefault {30};

    /**
     * @brief Default update rate (in Hz).
     */
    const float kUpdateRate {10.0};

    /**
     * @brief ROS topic that receives the raw (unfiltered) point cloud.
     */
    const std::string kPointCloudTopicRaw {"/camera/camera/depth/color/points"};

    /**
     * @brief ROS topic that publishes the filtered point cloud.
     */
    const std::string kPointCloudTopicFiltered {"/point_cloud/filtered"};

    /**
     * @brief Loads parameters from the ROS2 parameter server.
     * 
     * This function retrieves parameters such as thresholds for color filtering 
     * and the topics names for point cloud data from the ROS2 parameter server.
     */
    void load_parameters();

    /**
     * @brief Callback function for processing incoming point cloud data.
     * 
     * @param msg Shared pointer to the incoming PointCloud2 message.
     * 
     * This function is triggered whenever a new point cloud message is received 
     * on the subscribed topic. It processes the raw point cloud and applies 
     * filtering based on the thresholds defined.
     */
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    /**
     * @brief ROS2 subscription to receive PointCloud2 messages.
     * 
     * This subscription listens to a topic that provides raw point cloud data 
     * (sensor_msgs::msg::PointCloud2) and triggers the callback function 
     * to process the incoming data.
     */
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    /**
     * @brief ROS2 publisher to publish filtered PointCloud2 messages.
     * 
     * This publisher is used to send the processed and filtered point cloud data 
     * (sensor_msgs::msg::PointCloud2) to a specific topic.
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    /**
     * @brief Threshold value for red color filtering.
     * 
     * This value is used to filter the point cloud based on the red color component.
     */
    int red_threshold_ {};

    /**
     * @brief Threshold value for green color filtering.
     * 
     * This value is used to filter the point cloud based on the green color component.
     */
    int green_threshold_ {};

    /**
     * @brief Threshold value for blue color filtering.
     * 
     * This value is used to filter the point cloud based on the blue color component.
     */
    int blue_threshold_ {};

    /**
     * @brief Frame ID used for the point cloud data.
     * 
     * The frame ID indicates the coordinate frame in which the point cloud data is represented.
     * Default is "camera_link".
     */
    std::string frame_id_ {"camera_link"};

    /**
     * @brief ROS topic for receiving the raw point cloud data.
     * 
     * This topic is where the raw (unfiltered) point cloud is subscribed from.
     */
    std::string point_cloud_topic_raw_ {};

    /**
     * @brief ROS topic for publishing the filtered point cloud data.
     * 
     * This topic is where the filtered point cloud is published after processing.
     */
    std::string point_cloud_topic_filtered_ {};

    /**
     * @brief Update rate for processing the point cloud data.
     * 
     * This value controls how frequently the point cloud processing is updated, in Hz.
     */
    float update_rate_ {};

    /**
     * @brief Holds the filtered PointCloud2 message.
     * 
     * This message contains the filtered point cloud data that is published after processing.
     */
    sensor_msgs::msg::PointCloud2 filtered_msg_;
};

}  // namespace pointcloud_motion_capture

#endif  // POINT_CLOUD_READER_HPP
