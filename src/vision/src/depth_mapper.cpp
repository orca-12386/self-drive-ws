#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::placeholders::_4;

class DepthProcessorNode : public rclcpp::Node
{
public:
    DepthProcessorNode() : Node("depth_processor_node")
    {
        using namespace message_filters;

        // Subscribers
        depth_sub_.subscribe(this, "/zed_node/stereocamera/depth/image_raw");
        odom_sub_.subscribe(this, "odom");
        info_sub_.subscribe(this, "/zed_node/stereocamera/depth/camera_info");
        mask_sub_.subscribe(this, "image_mask");

        // Approximate Time Sync Policy
        sync_ = std::make_shared<Synchronizer<SyncPolicy>>(SyncPolicy(10), depth_sub_, odom_sub_, info_sub_, mask_sub_);
        sync_->registerCallback(std::bind(&DepthProcessorNode::callback, this, _1, _2, _3, _4));
    }

private:
    void callback(
        const sensor_msgs::msg::Image::ConstSharedPtr depth_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg,
        const sensor_msgs::msg::CameraInfo::ConstSharedPtr info_msg,
        const sensor_msgs::msg::Image::ConstSharedPtr mask_msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received synchronized messages");
        // Process depth, odometry, camera info, and mask here
    }

    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        nav_msgs::msg::Odometry,
        sensor_msgs::msg::CameraInfo,
        sensor_msgs::msg::Image>;

    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    message_filters::Subscriber<nav_msgs::msg::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> mask_sub_;

    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DepthProcessorNode>());
    rclcpp::shutdown();
    return 0;
}
