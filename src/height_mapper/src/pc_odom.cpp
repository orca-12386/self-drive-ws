#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

struct BotPosition {
    double x, y, z;
    double yaw;
};

class PointCloudTransformer : public rclcpp::Node {
public:
    PointCloudTransformer() : Node("point_cloud_transformer") {
        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed_node/stereocamera/points", 10,
            std::bind(&PointCloudTransformer::pointCloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&PointCloudTransformer::odomCallback, this, std::placeholders::_1));

        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/zed_node/transformed_points", 10);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    std::mutex odom_mutex_;
    BotPosition bot_pose;

    pcl::PointXYZRGB cloudPointToGlobalPoint(const pcl::PointXYZRGB &cloud_point,
                                           const BotPosition &bot_pos) {
        pcl::PointXYZRGB global_point;
        global_point.x = (bot_pos.x + cloud_point.x * sin(bot_pos.yaw) +
                         cloud_point.z * cos(bot_pos.yaw));
        global_point.y = (bot_pos.y + cloud_point.z * sin(bot_pos.yaw) -
                         cloud_point.x * cos(bot_pos.yaw));
        global_point.z = -1 * cloud_point.y + 1.5;
        global_point.r = cloud_point.r;
        global_point.g = cloud_point.g;
        global_point.b = cloud_point.b;
        return global_point;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        bot_pose.x = msg->pose.pose.position.x;
        bot_pose.y = msg->pose.pose.position.y;
        bot_pose.z = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, 
            msg->pose.pose.orientation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, bot_pose.yaw);
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::fromROSMsg(*msg, *input_cloud);
        
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(
            new pcl::PointCloud<pcl::PointXYZRGB>());

        BotPosition bot_pos;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            bot_pos = bot_pose;
        }

        for (const auto &point : input_cloud->points) {
            transformed_cloud->points.push_back(
                cloudPointToGlobalPoint(point, bot_pos));
        }

        transformed_cloud->header.frame_id = "map";
        transformed_cloud->width = transformed_cloud->points.size();
        transformed_cloud->height = 1;
        transformed_cloud->is_dense = true;

        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*transformed_cloud, output_msg);
        output_msg.header.stamp = this->now();
        transformed_cloud_pub_->publish(output_msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudTransformer>());
    rclcpp::shutdown();
    return 0;
}