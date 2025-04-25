#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

struct BotPosition {
  double x, y, z;
  double yaw;
};

class PointCloudTransformer : public rclcpp::Node {
public:
  PointCloudTransformer() : Node("point_cloud_transformer") {
    auto qos_pc =
        rclcpp::QoS(100).reliability(rclcpp::ReliabilityPolicy::Reliable);
    auto qos_odom = rclcpp::QoS(100).reliable();

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/zed_node/stereocamera/points", qos_pc,
        std::bind(&PointCloudTransformer::pointCloudCallback, this,
                  std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", qos_odom,
        std::bind(&PointCloudTransformer::odomCallback, this,
                  std::placeholders::_1));

    transformed_odom_cloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/nav/odom/point_cloud", qos_pc);

    transformed_base_link_cloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/nav/base_link/point_cloud", qos_pc);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      point_cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      transformed_odom_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      transformed_base_link_cloud_pub_;
  std::mutex odom_mutex_;
  BotPosition bot_pose;

  const float pitch = 23.5f * M_PI / 180.0f;
  const float cos_pitch = cos(pitch);
  const float sin_pitch = sin(pitch);

  pcl::PointXYZRGB cloudPointToOdom(const pcl::PointXYZRGB &cloud_point,
                                    const BotPosition &bot_pos) {
    pcl::PointXYZRGB global_point;
    const float yaw = bot_pos.yaw;
    const float cos_yaw = cos(yaw);
    const float sin_yaw = sin(yaw);

    const float cz_cp = cloud_point.z * cos_pitch;
    const float cy_sp = cloud_point.y * sin_pitch;

    global_point.x =
        bot_pos.x + (cz_cp - cy_sp) * cos_yaw + cloud_point.x * sin_yaw;
    global_point.y =
        bot_pos.y + (cz_cp - cy_sp) * sin_yaw - cloud_point.x * cos_yaw;
    global_point.z =
        1.5f - cloud_point.z * sin_pitch - cloud_point.y * cos_pitch;
    global_point.rgba = cloud_point.rgba;

    return global_point;
  }

  pcl::PointXYZRGB cloudPointToBaselink(const pcl::PointXYZRGB &cloud_point) {
    pcl::PointXYZRGB base_link_point;
    base_link_point.x = cloud_point.z * cos_pitch - cloud_point.y * sin_pitch;
    base_link_point.y = -cloud_point.x;
    base_link_point.z =
        1.5f - cloud_point.z * sin_pitch - cloud_point.y * cos_pitch;
    base_link_point.rgba = cloud_point.rgba;

    return base_link_point;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    bot_pose.x = msg->pose.pose.position.x;
    bot_pose.y = msg->pose.pose.position.y;
    bot_pose.z = msg->pose.pose.position.z;
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    double roll, pitch;
    tf2::Matrix3x3(q).getRPY(roll, pitch, bot_pose.yaw);
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*msg, *input_cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr odom_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr base_link_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>());

    BotPosition current_bot_pos;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      current_bot_pos = bot_pose;
    }

    for (const auto &point : input_cloud->points) {
      odom_cloud->points.push_back(cloudPointToOdom(point, current_bot_pos));
      base_link_cloud->points.push_back(cloudPointToBaselink(point));
    }

    odom_cloud->header.frame_id = "map";
    odom_cloud->width = msg->width; //odom_cloud->points.size() initially but that causes errors in the pc_mapper
    odom_cloud->height = msg->height;
    odom_cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2 odom_msg;
    pcl::toROSMsg(*odom_cloud, odom_msg);
    odom_msg.header.stamp = this->now();
    transformed_odom_cloud_pub_->publish(odom_msg);

    base_link_cloud->header.frame_id = "link_base";
    base_link_cloud->width = msg->width; //base_link_cloud->points.size() initially but causes issues with pc_mappper
    base_link_cloud->height = msg->height;
    base_link_cloud->is_dense = true;
    sensor_msgs::msg::PointCloud2 base_link_msg;
    pcl::toROSMsg(*base_link_cloud, base_link_msg);
    base_link_msg.header.stamp = this->now();
    transformed_base_link_cloud_pub_->publish(base_link_msg);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudTransformer>());
  rclcpp::shutdown();
  return 0;
}