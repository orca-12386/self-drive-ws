

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <memory>
#include <string>
#include <chrono>

using std::placeholders::_1;

class HeightMaskPublisherNode : public rclcpp::Node
{
public:
  HeightMaskPublisherNode()
  : Node("height_mask_publisher_node")
  {
    RCLCPP_INFO(this->get_logger(), "HeightMaskPublisherNode started");

    pc_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/zed_node/stereocamera/points",  
      rclcpp::QoS(10),
      std::bind(&HeightMaskPublisherNode::pcCallback, this, _1)
    );

    pc_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/points_filtered", 10
    );

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(40),
      std::bind(&HeightMaskPublisherNode::timerCallback, this)
    );
  }

private:
  void pcCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    latest_pc_ = msg;
    pc_received_ = true;
  }

  void timerCallback()
  {
    if (!pc_received_) {
      return;
    }
    filterPointCloud(latest_pc_);
  }

  void filterPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& input_cloud)
    {
        // Convert ROS2 PointCloud2 to PCL format with RGB
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*input_cloud, *pcl_cloud);

        // First: Z-axis filter
        pcl::PassThrough<pcl::PointXYZRGB> pass_z;
        pass_z.setInputCloud(pcl_cloud);
        pass_z.setFilterFieldName("z");
        pass_z.setFilterLimits(-std::numeric_limits<float>::infinity(), 7.0f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr z_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pass_z.filter(*z_filtered_cloud);

        // Second: Y-axis filter
        pcl::PassThrough<pcl::PointXYZRGB> pass_y;
        pass_y.setInputCloud(z_filtered_cloud);
        pass_y.setFilterFieldName("y");
        pass_y.setFilterLimits(-2.0f, -1.0f);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pass_y.filter(*final_filtered_cloud);

        // Convert back to ROS2 message
        sensor_msgs::msg::PointCloud2 output_cloud;
        pcl::toROSMsg(*final_filtered_cloud, output_cloud);
        output_cloud.header.stamp = input_cloud->header.stamp;
        output_cloud.header.frame_id = input_cloud->header.frame_id;
        pc_filtered_pub_->publish(output_cloud);

        RCLCPP_INFO(this->get_logger(), "Published filtered RGB point cloud with %zu points", final_filtered_cloud->size());
        pc_received_ = false;
    }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pc_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_filtered_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::PointCloud2::SharedPtr latest_pc_;
  bool pc_received_{false};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HeightMaskPublisherNode>());
  rclcpp::shutdown();
  return 0;
}
