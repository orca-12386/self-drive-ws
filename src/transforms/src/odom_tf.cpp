#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

class DynamicTFBroadcaster : public rclcpp::Node
{
public:
    DynamicTFBroadcaster()
    : Node("tf_odom_transformer")
    {
        // Subscriber to /dlo/odom_node/odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&DynamicTFBroadcaster::odom_callback, this, std::placeholders::_1)
        );

        // Publisher to /odom/transformed
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom/transformed", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Create and send TransformStamped
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "map";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = - msg->pose.pose.position.y; // Negate Y
        t.transform.translation.z = msg->pose.pose.position.z;

        t.transform.rotation = msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);

        // Create transformed Odometry message
        nav_msgs::msg::Odometry transformed_odom = *msg;
        transformed_odom.child_frame_id = "base_link";
        transformed_odom.header.frame_id = "map";
        transformed_odom.pose.pose.position.x *= 1.0;
        transformed_odom.pose.pose.position.y *= -1.0;  // Invert Y position
        transformed_odom.twist.twist.linear.y *= -1.0;  // Invert Y velocity
        transformed_odom.twist.twist.linear.x *= 1.0;  

        odom_pub_->publish(transformed_odom);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}