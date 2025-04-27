#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include <deque>
#include <memory>

using namespace std::chrono_literals;

class LidarTfNode : public rclcpp::Node {

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_laserscan_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pointcloud_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_laserscan_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time last_pointcloud_time_;
    rclcpp::Time last_laserscan_time_;
    rclcpp::Time last_imu_time_;
    
    bool received_pointcloud_ = false;
    bool received_laserscan_ = false;
    bool received_imu_ = false;
    
    int imu_window_size_;
    
    struct Vector3Sum { double x=0, y=0, z=0; };
    struct QuaternionSum { double x=0, y=0, z=0, w=0; };
    
    Vector3Sum linear_acc_sum_;
    Vector3Sum angular_vel_sum_;
    QuaternionSum orientation_sum_;
    
    std::deque<geometry_msgs::msg::Vector3> linear_acc_window_;
    std::deque<geometry_msgs::msg::Vector3> angular_vel_window_;
    std::deque<geometry_msgs::msg::Quaternion> orientation_window_;
public:

  LidarTfNode() : Node("lidar_tf") {
    // Publishers
    pub_pointcloud_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", 10);
    pub_laserscan_ = create_publisher<sensor_msgs::msg::LaserScan>("/lidar/scan", 10);
    pub_imu_ = create_publisher<sensor_msgs::msg::Imu>("/lidar/imu", 10);

    // Subscribers
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
// sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
//     "/ouster/points", qos,
//     std::bind(&LidarTfNode::pointCloudCallback, this, std::placeholders::_1));

    sub_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/ouster/points", qos,
      std::bind(&LidarTfNode::pointCloudCallback, this, std::placeholders::_1));
    
    sub_laserscan_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/ouster/scan", 10,
      std::bind(&LidarTfNode::laserScanCallback, this, std::placeholders::_1));
    
    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "/ouster/imu", 10,
      std::bind(&LidarTfNode::imuCallback, this, std::placeholders::_1));

    // Timer for data reception checks
    timer_ = create_wall_timer(
      500ms, std::bind(&LidarTfNode::checkDataReception, this));

    // declare_parameter("imu_window_size", 10);
    // imu_window_size_ = get_parameter("imu_window_size").as_int();
      imu_window_size_ = 10;
    RCLCPP_INFO(get_logger(), 
      "Lidar TF node started with IMU smoothing. Window size: %d", imu_window_size_);
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    received_pointcloud_ = true;
    last_pointcloud_time_ = now();
    auto new_msg = *msg;
    new_msg.header.stamp = now();
    pub_pointcloud_->publish(new_msg);
  }

  void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    received_laserscan_ = true;
    last_laserscan_time_ = now();
    auto new_msg = *msg;
    new_msg.header.stamp = now();
    pub_laserscan_->publish(new_msg);
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    received_imu_ = true;
    last_imu_time_ = now();

    // Update rolling sums
    linear_acc_sum_.x += msg->linear_acceleration.x;
    linear_acc_sum_.y += msg->linear_acceleration.y;
    linear_acc_sum_.z += msg->linear_acceleration.z;

    angular_vel_sum_.x += msg->angular_velocity.x;
    angular_vel_sum_.y += msg->angular_velocity.y;
    angular_vel_sum_.z += msg->angular_velocity.z;

    orientation_sum_.x += msg->orientation.x;
    orientation_sum_.y += msg->orientation.y;
    orientation_sum_.z += msg->orientation.z;
    orientation_sum_.w += msg->orientation.w;

    // Store in window
    linear_acc_window_.push_back(msg->linear_acceleration);
    angular_vel_window_.push_back(msg->angular_velocity);
    orientation_window_.push_back(msg->orientation);

    // Maintain window size
    if (linear_acc_window_.size() > imu_window_size_) {
      const auto& old = linear_acc_window_.front();
      linear_acc_sum_.x -= old.x;
      linear_acc_sum_.y -= old.y;
      linear_acc_sum_.z -= old.z;
      linear_acc_window_.pop_front();

      const auto& old_ang = angular_vel_window_.front();
      angular_vel_sum_.x -= old_ang.x;
      angular_vel_sum_.y -= old_ang.y;
      angular_vel_sum_.z -= old_ang.z;
      angular_vel_window_.pop_front();

      const auto& old_ori = orientation_window_.front();
      orientation_sum_.x -= old_ori.x;
      orientation_sum_.y -= old_ori.y;
      orientation_sum_.z -= old_ori.z;
      orientation_sum_.w -= old_ori.w;
      orientation_window_.pop_front();
    }

    // Publish averaged IMU
    auto new_msg = *msg;
    new_msg.header.stamp = now();
    
    if (linear_acc_window_.size() >= 3) {
      size_t window_size = linear_acc_window_.size();
      new_msg.linear_acceleration = getAveragedVector3(linear_acc_sum_, window_size);
      new_msg.angular_velocity = getAveragedVector3(angular_vel_sum_, window_size);
      new_msg.orientation = getAveragedQuaternion(orientation_sum_, window_size);
    }
    
    pub_imu_->publish(new_msg);
  }

  void checkDataReception() {
    auto current_time = now();
    const double timeout = 1.0;

    if (!received_pointcloud_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No PointCloud data received on /ouster/points");
    } else if ((current_time - last_pointcloud_time_).seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No PointCloud data for %.1f seconds",
                         (current_time - last_pointcloud_time_).seconds());
    }

    if (!received_laserscan_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No LaserScan data received on /ouster/scan");
    } else if ((current_time - last_laserscan_time_).seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No LaserScan data for %.1f seconds",
                         (current_time - last_laserscan_time_).seconds());
    }

    if (!received_imu_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No IMU data received on /ouster/imu");
    } else if ((current_time - last_imu_time_).seconds() > timeout) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "No IMU data for %.1f seconds",
                         (current_time - last_imu_time_).seconds());
    }
  }

  geometry_msgs::msg::Quaternion getAveragedQuaternion(const QuaternionSum& sum, size_t size) {
    geometry_msgs::msg::Quaternion avg;
    avg.x = sum.x / size;
    avg.y = sum.y / size;
    avg.z = sum.z / size;
    avg.w = sum.w / size;

    double norm = sqrt(avg.x*avg.x + avg.y*avg.y + avg.z*avg.z + avg.w*avg.w);
    avg.x /= norm;
    avg.y /= norm;
    avg.z /= norm;
    avg.w /= norm;
    return avg;
  }

  geometry_msgs::msg::Vector3 getAveragedVector3(const Vector3Sum& sum, size_t size) {
    geometry_msgs::msg::Vector3 avg;
    avg.x = sum.x / size;
    avg.y = sum.y / size;
    avg.z = sum.z / size;
    return avg;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarTfNode>());
  rclcpp::shutdown();
  return 0;
}
