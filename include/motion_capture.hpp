// Copyright 2024 João Vitor Silva Mendes

#ifndef MOTION_CAPTURE_HPP_
#define MOTION_CAPTURE_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <queue>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

namespace pointcloud_motion_capture
{
class MotionCapture : public rclcpp::Node
{
public:
  /**
   * @brief Enumeration representing indices for the centroid coordinates.
   *
   * This enum defines indices for accessing the X, Y, and Z coordinates of the
   * centroid in the Eigen::Vector4f. These indices help in improving the
   * readability when accessing specific dimensions of the centroid vector.
   */
  enum CentroidIndex
  {
    kCentroidX = 0,  ///< Index for the X-coordinate of the centroid.
    kCentroidY = 1,  ///< Index for the Y-coordinate of the centroid.
    kCentroidZ = 2   ///< Index for the Z-coordinate of the centroid.
  };

  /**
   * @brief Enumeration representing indices for the velocity vector.
   *
   * This enum defines indices for accessing the X, Y, and Z components of a
   * velocity vector in the Eigen::Vector3f. These indices help improve readability
   * when accessing specific dimensions of the velocity vector.
   */
  enum VectorIndex
  {
    kVectorX = 0,  ///< Index for the X-component of the velocity vector.
    kVectorY = 1,  ///< Index for the Y-component of the velocity vector.
    kVectorZ = 2   ///< Index for the Z-component of the velocity vector.
  };

  /**
   * @brief Default topic name for subscribing to the filtered point cloud data.
   *
   * This string stores the default ROS topic name for the filtered point cloud
   * data, which the node subscribes to in order to process the point cloud
   * and extract relevant object information.
   */
  std::string kDefaultPointCloudTopicFiltered {"/point_cloud/filtered"};

  /**
   * @brief Default topic name for publishing the calculated object position.
   *
   * This string stores the default ROS topic name where the calculated object
   * position (as geometry_msgs::msg::Point) is published.
   */
  std::string kDefaultPositionPublishTopic {"/object_position"};

  /**
   * @brief Default topic name for publishing the calculated object velocity.
   *
   * This string stores the default ROS topic name where the calculated object
   * velocity (as geometry_msgs::msg::Vector3) is published.
   */
  std::string kDefaultVelocityPublishTopic {"/object_velocity"};

  /**
   * @brief Default topic name for publishing the calculated object acceleration.
   *
   * This string stores the default ROS topic name where the calculated object
   * acceleration (as geometry_msgs::msg::Vector3) is published.
   */
  std::string kDefaultAccelerationPublishTopic {"/object_acceleration"};

  /**
   * @brief Default number of position samples used for calculations.
   *
   * This integer value defines the default number of recent position samples
   * stored in the position queue and used for calculating the filtered position
   * of the object.
   */
  int kDefaultPositionSamples{5};

  /**
   * @brief Default number of velocity samples used for calculations.
   *
   * This integer value defines the default number of recent velocity samples
   * stored in the velocity queue and used for calculating the filtered velocity
   * of the object.
   */
  int kDefaultVelocitySamples{5};

  /**
   * @brief Default publish rate in Hz for publishing position, velocity, and acceleration.
   *
   * This double value defines the default rate, in Hz, at which the position,
   * velocity, and acceleration data are published to their respective ROS topics.
   */
  double kDefaultPublishRate{10.0};

  /**
   * @brief Default queue size for ROS publishers and subscriptions.
   *
   * This integer value defines the default queue size for both publishers and
   * subscriptions, controlling how many messages are stored before being processed
   * or discarded.
   */
  int kDefaultQueueSize{10};

  /**
   * @brief Conversion factor from seconds to milliseconds.
   *
   * This double value defines the constant factor for converting seconds to
   * milliseconds. It is used to convert the publish rate or other time-related
   * values in calculations that require time-based units.
   */
  double kMillisecondsPerSecond{1000.0};

  /**
   * @brief Constructor for the MotionCapture class.
   *
   * This constructor initializes the MotionCapture node, setting up the
   * necessary ROS2 publishers, subscriptions, and timers. It also initializes
   * default values for position, velocity, and other parameters.
   */
  MotionCapture();

private:
  /**
   * @brief Callback function for processing incoming PointCloud2 messages.
   *
   * This function is triggered whenever a new PointCloud2 message is received
   * on the subscribed topic. It processes the point cloud data to extract the
   * centroid of the object and updates the position samples for further
   * calculations.
   *
   * @param msg Shared pointer to the received sensor_msgs::msg::PointCloud2 message.
   */
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  /**
   * @brief Updates the position samples queue with a new centroid value.
   *
   * This function adds the given centroid value to the position samples queue.
   * The queue is used to store recent position data for calculating the filtered
   * position of the object.
   *
   * @param centroid The centroid of the object in 3D space, represented as Eigen::Vector4f.
   */
  void updatePositionSamples(const Eigen::Vector4f & centroid);

  /**
   * @brief Retrieves the filtered position of the object.
   *
   * This function calculates and returns the filtered position of the object
   * based on the stored position samples. The filtering is typically done by
   * averaging or applying a smoothing algorithm to the samples in the queue.
   *
   * @return The filtered position as Eigen::Vector4f.
   */
  Eigen::Vector4f getFilteredPosition();

  /**
   * @brief Updates the velocity samples queue with a new velocity value.
   *
   * This function adds the given velocity value to the velocity samples queue.
   * The queue is used to store recent velocity data for calculating the filtered
   * velocity of the object.
   *
   * @param velocity The velocity of the object in 3D space, represented as Eigen::Vector3f.
   */
  void updateVelocitySamples(const Eigen::Vector3f & velocity);

  /**
   * @brief Retrieves the filtered velocity of the object.
   *
   * This function calculates and returns the filtered velocity of the object
   * based on the stored velocity samples. The filtering is typically done by
   * averaging or applying a smoothing algorithm to the samples in the queue.
   *
   * @return The filtered velocity as Eigen::Vector3f.
   */
  Eigen::Vector3f getFilteredVelocity();

  /**
   * @brief Publishes the calculated position, velocity, and acceleration.
   *
   * This function is triggered by a ROS2 timer and publishes the calculated
   * position, velocity, and acceleration data at regular intervals. The data
   * is published to the respective ROS topics as geometry_msgs messages.
   */
  void publishData();

  /**
   * @brief Publishes the object's position to a ROS topic.
   *
   * This function takes the object's centroid as input and publishes its position
   * to the specified ROS topic as a geometry_msgs::msg::Point message.
   *
   * @param centroid The centroid of the object in 3D space, represented as Eigen::Vector4f.
   */
  void publishPosition(const Eigen::Vector4f & centroid);

  /**
   * @brief Publishes the object's velocity to a ROS topic.
   *
   * This function calculates the object's velocity based on the current position
   * and the previous position. It then publishes the calculated velocity as a
   * geometry_msgs::msg::Vector3 message.
   *
   * @param current_position The current position of the object in 3D space, represented as Eigen::Vector4f.
   * @return The calculated velocity as Eigen::Vector3f.
   */
  Eigen::Vector3f publishVelocity(const Eigen::Vector4f & current_position);

  /**
   * @brief Publishes the object's acceleration to a ROS topic.
   *
   * This function calculates the object's acceleration based on the current
   * velocity and the previous velocity. It then publishes the calculated
   * acceleration as a geometry_msgs::msg::Vector3 message.
   *
   * @param current_velocity The current velocity of the object in 3D space, represented as Eigen::Vector3f.
   */
  void publishAcceleration(const Eigen::Vector3f & current_velocity);

  /**
   * @brief Loads parameters from the ROS2 parameter server.
   *
   * This function retrieves the necessary parameters from the ROS2 parameter server
   * to configure the node. Parameters include topics for publishing and subscribing,
   * the number of samples for position and velocity calculations, the queue size, and
   * the publish rate.
   */
  void loadParameters();

  /**
   * @brief ROS2 subscription to receive PointCloud2 messages.
   *
   * This subscription listens to a topic that provides filtered point cloud data
   * (sensor_msgs::msg::PointCloud2) and triggers the callback function to process
   * the incoming data, allowing the system to extract relevant information for further use.
   */
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

  /**
   * @brief ROS2 publisher to send the calculated object position.
   *
   * This publisher sends geometry_msgs::msg::Point messages, representing
   * the estimated position of an object derived from the point cloud data.
   * The position is calculated based on the centroid of the object in the point cloud.
   */
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_publisher_;

  /**
   * @brief ROS2 publisher to send the calculated object velocity.
   *
   * This publisher sends geometry_msgs::msg::Vector3 messages, representing
   * the estimated velocity of an object. The velocity is computed as the
   * change in position over time, based on sequential position samples.
   */
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr velocity_publisher_;

  /**
   * @brief ROS2 publisher to send the calculated object acceleration.
   *
   * This publisher sends geometry_msgs::msg::Vector3 messages, representing
   * the estimated acceleration of an object. The acceleration is computed
   * as the change in velocity over time, based on sequential velocity samples.
   */
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr acceleration_publisher_;

  /**
   * @brief Queue to store recent position samples for calculating position.
   *
   * This queue holds Eigen::Vector4f values representing the object’s position
   * in the 3D space. The queue size is limited based on the number of position
   * samples specified in the configuration, and is used to compute smoothed
   * or averaged position values over time.
   */
  std::queue<Eigen::Vector4f> position_queue_;

  /**
   * @brief Queue to store recent velocity samples for calculating velocity.
   *
   * This queue holds Eigen::Vector3f values representing the object’s velocity
   * in the 3D space. The velocity is calculated based on the change in position
   * between consecutive samples stored in this queue.
   */
  std::queue<Eigen::Vector3f> velocity_queue_;

  /**
   * @brief The previous position of the object in 3D space.
   *
   * This Eigen::Vector4f value stores the last known position of the object,
   * which is used to calculate velocity when new position data is received.
   */
  Eigen::Vector4f previous_position_;

  /**
   * @brief The previous velocity of the object in 3D space.
   *
   * This Eigen::Vector3f value stores the last known velocity of the object,
   * which is used to calculate acceleration when new velocity data is received.
   */
  Eigen::Vector3f previous_velocity_;

  /**
   * @brief The timestamp of the previous position sample.
   *
   * This rclcpp::Time object stores the time at which the last position sample
   * was recorded. It is used to compute the time difference between consecutive
   * position updates, which is essential for velocity calculations.
   */
  rclcpp::Time previous_time_;

  /**
   * @brief The timestamp of the previous velocity sample.
   *
   * This rclcpp::Time object stores the time at which the last velocity sample
   * was recorded. It is used to compute the time difference between consecutive
   * velocity updates, which is essential for acceleration calculations.
   */
  rclcpp::Time previous_velocity_time_;

  /**
   * @brief Number of position samples used for calculations.
   *
   * This integer value defines how many recent position samples are stored in the
   * position queue and used for calculating smoothed or averaged position data.
   */
  int position_samples_;

  /**
   * @brief Number of velocity samples used for calculations.
   *
   * This integer value defines how many recent velocity samples are stored in the
   * velocity queue and used for calculating smoothed or averaged velocity data.
   */
  int velocity_samples_;

  /**
   * @brief Queue size for message publishers and subscriptions.
   *
   * This integer value specifies the queue size for both publishers and subscriptions,
   * controlling how many messages are stored before being processed or discarded.
   */
  int queue_size_;

  /**
   * @brief Publish rate for sending position, velocity, and acceleration data.
   *
   * This double value specifies the rate, in Hz, at which the publishers send
   * the calculated position, velocity, and acceleration messages. It controls
   * the frequency of updates for the published data.
   */
  double publish_rate_;

  /**
   * @brief Timer used for scheduling the publication of data.
   *
   * This ROS2 timer triggers the publication of position, velocity, and acceleration
   * data at the rate specified by publish_rate_. The timer ensures regular updates
   * based on the configured time interval.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Topic name for subscribing to filtered point cloud data.
   *
   * This string value holds the name of the topic that the node subscribes to,
   * which provides filtered point cloud data for processing and extracting object
   * information.
   */
  std::string point_cloud_topic_filtered_;

  /**
   * @brief Topic name for publishing the calculated position of an object.
   *
   * This string value holds the name of the topic that the node publishes
   * calculated position data to, as geometry_msgs::msg::Point messages.
   */
  std::string position_publish_topic_;

  /**
   * @brief Topic name for publishing the calculated velocity of an object.
   *
   * This string value holds the name of the topic that the node publishes
   * calculated velocity data to, as geometry_msgs::msg::Vector3 messages.
   */
  std::string velocity_publish_topic_;

  /**
   * @brief Topic name for publishing the calculated acceleration of an object.
   *
   * This string value holds the name of the topic that the node publishes
   * calculated acceleration data to, as geometry_msgs::msg::Vector3 messages.
   */
  std::string acceleration_publish_topic_;
};

}  // namespace pointcloud_motion_capture

#endif  // MOTION_CAPTURE_HPP_
