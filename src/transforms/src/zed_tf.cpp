#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <deque>

using namespace std::chrono_literals;

class ImuTransformer : public rclcpp::Node {
public:
  ImuTransformer() : Node("zed_tf") {
    // Parameters
    declare_parameter("window_size", 50);
    window_size_ = get_parameter("window_size").as_int();

    // Subscribers
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "/zed_node/imu/data", 10,
      std::bind(&ImuTransformer::imuCallback, this, std::placeholders::_1));

    // TF Broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Monitoring
    monitor_timer_ = create_wall_timer(
      500ms, std::bind(&ImuTransformer::monitorCallback, this));

    RCLCPP_INFO(get_logger(), "IMU Transformer initialized. Window size: %zu", window_size_);
  }

private:
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    received_imu_ = true;
    last_imu_time_ = now();

    // Process orientation
    tf2::Quaternion imu_orientation(
      msg->orientation.x,
      msg->orientation.y,
      msg->orientation.z,
      msg->orientation.w
    );

    double roll, pitch, yaw;
    tf2::Matrix3x3(imu_orientation).getRPY(roll, pitch, yaw);

    // Update pitch window
    if (pitch_window_.size() >= window_size_) {
      running_sum_ -= pitch_window_.front();
      pitch_window_.pop_front();
    }
    
    pitch_window_.push_back(pitch);
    running_sum_ += pitch;
    double avg_pitch = running_sum_ / pitch_window_.size();

    // Create transform
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = "base_link";
    transform.child_frame_id = "zed_camera_link";

    tf2::Quaternion corrected_orientation;
    corrected_orientation.setRPY(roll, avg_pitch, 0.0);
    transform.transform.rotation.x = corrected_orientation.x();
    transform.transform.rotation.y = corrected_orientation.y();
    transform.transform.rotation.z = corrected_orientation.z();
    transform.transform.rotation.w = corrected_orientation.w();

    transform.transform.translation.x = -0.114;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 1.0;

    tf_broadcaster_->sendTransform(transform);
  }

  void monitorCallback() {
    auto current_time = now();
    
    if (!received_imu_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No IMU data received from /zed_node/imu/data");
    }
    else if ((current_time - last_imu_time_).seconds() > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No IMU data for %.1f seconds", 
        (current_time - last_imu_time_).seconds());
    }

    if (pitch_window_.size() < window_size_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Pitch averaging window not full (%zu/%zu)",
        pitch_window_.size(), window_size_);
    }
  }

  // Member variables
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
  
  rclcpp::Time last_imu_time_;
  bool received_imu_ = false;
  size_t window_size_;
  std::deque<double> pitch_window_;
  double running_sum_ = 0.0;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuTransformer>());
  rclcpp::shutdown();
  return 0;
}
