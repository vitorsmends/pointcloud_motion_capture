#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <queue>
#include <Eigen/Core>

class PointCloudProcessor : public rclcpp::Node
{
public:
    PointCloudProcessor() : Node("point_cloud_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/point_cloud/filtered", 10, std::bind(&PointCloudProcessor::pointCloudCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("object_position", 10);
    }

private:
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *cloud);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid(*cloud, centroid);

        publishPosition(centroid);
    }

    void publishPosition(const Eigen::Vector4f &centroid)
    {
        geometry_msgs::msg::Point position_msg;
        position_msg.x = centroid[0];
        position_msg.y = centroid[1];
        position_msg.z = centroid[2];

        publisher_->publish(position_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudProcessor>());
    rclcpp::shutdown();
    return 0;
}
