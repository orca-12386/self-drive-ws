// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <topic_remapper/topic_remapper.hpp>
#include "topic_remapper/srv/change_topic.hpp"

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>
#include <map>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicRemapperNode<geometry_msgs::msg::PoseStamped>>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
