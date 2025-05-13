// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <memory>
#include <string>
#include <cmath>


class MultiMapAssemblerNode : public rclcpp::Node
{
public:
    MultiMapAssemblerNode() : 
    rclcpp::Node("multimap_assembler_node")
    {
        this->declare_parameter("map1_sub_topic", rclcpp::PARAMETER_STRING);
        this->declare_parameter("map2_sub_topic", rclcpp::PARAMETER_STRING);
        this->declare_parameter("map_pub_topic", rclcpp::PARAMETER_STRING);
        this->declare_parameter("assemble_mode", rclcpp::PARAMETER_INTEGER);

        RCLCPP_INFO(this->get_logger(), "multimap_assembler_node started");

        this->initialise_data();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&MultiMapAssemblerNode::timer_callback, this));
    };

private:
    void initialise_data() {
        std::string map1_sub_topic = this->get_parameter("map1_sub_topic").as_string();
        std::string map2_sub_topic = this->get_parameter("map2_sub_topic").as_string();
        std::string map_pub_topic = this->get_parameter("map_pub_topic").as_string();
        assemble_mode = this->get_parameter("assemble_mode").as_int();
        map1_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map1_sub_topic, 10, std::bind(&MultiMapAssemblerNode::map1Callback, this, std::placeholders::_1));
        map2_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map2_sub_topic, 10, std::bind(&MultiMapAssemblerNode::map2Callback, this, std::placeholders::_1));
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, 10);
        map1_recv = false;
        map2_recv = false;
        map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        map_msg->header.frame_id = "robot/odom";
    }

    void map1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map1_msg = msg;
        map1_recv = true;
    }

    void map2Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map2_msg = msg;
        map2_recv = true;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    void assemble_and_publish_map(nav_msgs::msg::OccupancyGrid::SharedPtr map1, nav_msgs::msg::OccupancyGrid::SharedPtr map2) {   
        if(map_msg->data.size() != map1->info.width * map1->info.height) {
            map_msg->data.resize(map1->info.width * map1->info.height, -1);
        } 
        std::fill(std::begin(map_msg->data), std::end(map_msg->data), -1);
        map_msg->info.width = map1->info.width;
        map_msg->info.height = map1->info.height;
        map_msg->info.resolution = map1->info.resolution;
        map_msg->info.origin.position.x = map1->info.origin.position.x;
        map_msg->info.origin.position.y = map1->info.origin.position.y;
        map_msg->info.origin.position.z = map1->info.origin.position.z;
        map_msg->info.origin.orientation.w = map1->info.origin.orientation.w;

        int index;
        for(int i=0;i<map_msg->info.height;i++) {
            for(int j=0;j<map_msg->info.width;j++) {
                index = (i*map_msg->info.width) + j;
                if(assemble_mode == 1) {
                    if(map1->data[index] > -1) {
                        map_msg->data[index] = map1->data[index];
                    }                    
                    if(map2->data[index] > map_msg->data[index]) {
                        map_msg->data[index] = map2->data[index];
                    }
                } else {
                    if(map1->data[index] > -1 && !(map2->data[index] > 0)) {
                        map_msg->data[index] = map1->data[index];
                    }
                }
            }
        }

        map_msg->header.stamp = this->now();
        map_pub->publish(*map_msg);
    }

    void timer_callback() {
        if(map1_recv && map2_recv) {
            assemble_and_publish_map(map1_msg, map2_msg);
        } else {
            log("Waiting for subscriptions");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map1_sub, map2_sub;
    bool map1_recv;
    bool map2_recv;
    int assemble_mode;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, map1_msg, map2_msg;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiMapAssemblerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
