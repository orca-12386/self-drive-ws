#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

using namespace std::chrono_literals;

class PointCloudTransformer : public rclcpp::Node {

  // Member variables
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr monitor_timer_;
    
    rclcpp::Time last_pointcloud_time_;
    rclcpp::Time last_successful_transform_time_;
    bool received_pointcloud_ = false;
    bool transform_available_ = false;
    std::string target_frame_;
public:
  PointCloudTransformer() : Node("odom_pc") {
    // Parameters

    this->declare_parameter<std::string>("pointcloud_topic", "/zed/zed_node/point_cloud/cloud_registered");
    this->declare_parameter<std::string>("target_frame", "robot/odom");
    this->declare_parameter<std::string>("pc_pub_topic", "/nav/point_cloud");

    
    std::string pc_pub_topic = this->get_parameter("pc_pub_topic").as_string();
    std::string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    

    //For real world, pc topic: "/zed/zed_node/point_cloud/cloud_registered"

    // Publishers/Subscribers
    pc_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(pc_pub_topic, 10);
    pc_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic, 10,
      std::bind(&PointCloudTransformer::pointcloudCallback, this, std::placeholders::_1));
    //pc topic for sim /zed_node/stereocamera/points

    // TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Monitoring
    monitor_timer_ = create_wall_timer(
      500ms, std::bind(&PointCloudTransformer::monitorCallback, this));

    RCLCPP_INFO(get_logger(), "PointCloud Transformer initialized. Waiting for data...");
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    received_pointcloud_ = true;
    last_pointcloud_time_ = now();

    try {
      auto transform = tf_buffer_->lookupTransform(
        target_frame_, cloud_msg->header.frame_id, cloud_msg->header.stamp,
        1s);

      auto transformed_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
      tf2::doTransform(*cloud_msg, *transformed_cloud, transform);
      transformed_cloud->header.stamp = now();
      transformed_cloud->header.frame_id = target_frame_;
      
      pc_pub_->publish(std::move(transformed_cloud));
      last_successful_transform_time_ = now();
      transform_available_ = true;
    } 
    catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "Could not transform pointcloud: %s", ex.what());
      transform_available_ = false;
    }
  }

  void monitorCallback() {
    auto current_time = now();
    
    if (!received_pointcloud_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No PointCloud data received from Zed");
    }
    else if ((current_time - last_pointcloud_time_).seconds() > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No PointCloud data for %.1f seconds", 
        (current_time - last_pointcloud_time_).seconds());
    }

    if (!transform_available_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Transform to %s not available", target_frame_.c_str());
    }
    else if ((current_time - last_successful_transform_time_).seconds() > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No successful transform for %.1f seconds",
        (current_time - last_successful_transform_time_).seconds());
    }
  }

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}
