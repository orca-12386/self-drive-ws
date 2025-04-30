#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <chrono>

using namespace std::chrono_literals;

class SensorRepublisher : public rclcpp::Node
{
public:
    SensorRepublisher()
    : Node("sensor_republisher")
    {
        auto qos = rclcpp::SensorDataQoS();
        auto qos_reliable = rclcpp::SensorDataQoS().reliable();
        // PointCloud2
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", qos,
            std::bind(&SensorRepublisher::pointcloud_callback, this, std::placeholders::_1));
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar/points", qos_reliable);

        // LaserScan
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/ouster/scan", qos,
            std::bind(&SensorRepublisher::scan_callback, this, std::placeholders::_1));
        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/lidar/scan", qos_reliable);

        // IMU
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/ouster/imu", qos,
            std::bind(&SensorRepublisher::imu_callback, this, std::placeholders::_1));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/lidar/imu", qos_reliable);

        // Timer to check for data
        watchdog_timer_ = this->create_wall_timer(1s, std::bind(&SensorRepublisher::check_data_timeout, this));

        RCLCPP_INFO(this->get_logger(), "Sensor republisher initialized.");
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        last_pointcloud_time_ = this->now();
        auto modified = *msg;
        modified.header.stamp = this->now();
        pointcloud_pub_->publish(modified);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_time_ = this->now();
        auto modified = *msg;
        modified.header.stamp = this->now();
        scan_pub_->publish(modified);
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        last_imu_time_ = this->now();
        auto modified = *msg;
        modified.header.stamp = this->now();
        imu_pub_->publish(modified);
    }

    void check_data_timeout()
    {
        auto now = this->now();
        rclcpp::Duration timeout = rclcpp::Duration::from_seconds(5.0);

        if ((now - last_pointcloud_time_) > timeout)
            RCLCPP_WARN(this->get_logger(), "No PointCloud2 data received in the last 5 seconds.");
        if ((now - last_scan_time_) > timeout)
            RCLCPP_WARN(this->get_logger(), "No LaserScan data received in the last 5 seconds.");
        if ((now - last_imu_time_) > timeout)
            RCLCPP_WARN(this->get_logger(), "No IMU data received in the last 5 seconds.");
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    // Watchdog timer
    rclcpp::TimerBase::SharedPtr watchdog_timer_;

    // Last message timestamps
    rclcpp::Time last_pointcloud_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_scan_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_imu_time_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorRepublisher>());
    rclcpp::shutdown();
    return 0;
}
