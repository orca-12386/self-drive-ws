// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "topic_remapper/srv/change_topic.hpp"

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>
#include <map>



template<typename message_type>
class TopicRemapperNode : public rclcpp::Node {
public:
    TopicRemapperNode() : rclcpp::Node("topic_remapper_node") {
        RCLCPP_INFO(this->get_logger(), "topic_remapper_node started");

        this->declare_parameter("default_sub_topic", rclcpp::PARAMETER_STRING);
        this->declare_parameter("pub_topic", rclcpp::PARAMETER_STRING);

        this->initialise_data();
    };

private:
    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    void change_topic(const std::shared_ptr<topic_remapper::srv::ChangeTopic::Request> request) {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), request->new_topic.c_str());
        sub.reset();
        sub = this->create_subscription<message_type>(
            std::string(request->new_topic), 10, std::bind(&TopicRemapperNode::topic_callback, this, std::placeholders::_1));
        return;
    }

    void initialise_data() {
        std::string default_sub_topic = this->get_parameter("default_sub_topic").as_string();
        std::string pub_topic = this->get_parameter("pub_topic").as_string();

        sub = this->create_subscription<message_type>(default_sub_topic, 10, std::bind(&TopicRemapperNode::topic_callback, this, std::placeholders::_1));
        pub = this->create_publisher<message_type>(pub_topic, 10);
        change_topic_service = this->create_service<topic_remapper::srv::ChangeTopic>("change_topic", std::bind(&TopicRemapperNode::change_topic, this, std::placeholders::_1));
    }

    void topic_callback(std::shared_ptr<message_type> msg) {
        pub->publish(*msg);
    }

    typename rclcpp::Subscription<message_type>::SharedPtr sub;
    typename rclcpp::Publisher<message_type>::SharedPtr pub;
    rclcpp::Service<topic_remapper::srv::ChangeTopic>::SharedPtr change_topic_service;

    rclcpp::TimerBase::SharedPtr timer;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TopicRemapperNode<nav_msgs::msg::OccupancyGrid>>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
