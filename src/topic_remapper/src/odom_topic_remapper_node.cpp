// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

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
    auto node = std::make_shared<TopicRemapperNode<nav_msgs::msg::Odometry>>("/topic_remapper/odometry");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
