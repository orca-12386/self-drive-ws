#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class PointCloudFilter : public rclcpp::Node {
public:
  PointCloudFilter() : Node("pointcloud_filter") {
    // Declare and get parameters
    this->declare_parameter("robot_radius", 0.5);
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    
    // Create subscribers and publishers
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_sub", 
      rclcpp::QoS(1),
      std::bind(&PointCloudFilter::filterCloud, this, std::placeholders::_1));
      
    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "obstacle_cloud_pub", 
      rclcpp::QoS(1));
      
    RCLCPP_INFO(this->get_logger(), "PointCloud Filter initialized with robot radius: %f", robot_radius_);
  }

  void filterCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    
    pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
    for (size_t i = 0; i < cloud.size(); i++) {
      float distance = std::sqrt(cloud.points[i].x * cloud.points[i].x +
                               cloud.points[i].y * cloud.points[i].y +
                               cloud.points[i].z * cloud.points[i].z);
      if (distance > robot_radius_) {
        filtered_cloud.push_back(cloud[i]);
      }
    }
    
    auto filtered_msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(filtered_cloud, *filtered_msg);
    filtered_msg->header = cloud_msg->header;
    pub_->publish(std::move(*filtered_msg));
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  double robot_radius_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFilter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}