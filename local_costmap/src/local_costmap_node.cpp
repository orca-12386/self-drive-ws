// local_costmap_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <fstream>

class LocalCostmapPublisher : public rclcpp::Node
{
public:
    LocalCostmapPublisher() : Node("local_costmap_publisher")
    {
        this->declare_parameter("map_sub_topic", rclcpp::PARAMETER_STRING);
        std::string map_sub_topic = this->get_parameter("map_sub_topic").as_string();

        // Initialize tf2 buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Load parameters from YAML file
        loadParams();

        // Subscribe to global map and robot pose
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_sub_topic, 10, std::bind(&LocalCostmapPublisher::mapCallback, this, std::placeholders::_1));
        
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot_pose_global", 10, std::bind(&LocalCostmapPublisher::poseCallback, this, std::placeholders::_1));

        // Publisher for local costmap
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            map_sub_topic+std::string("/local"), 10);

        RCLCPP_INFO(this->get_logger(), "Local Costmap Publisher initialized");
    }

private:
    void loadParams()
    {
        // Read YAML file
        std::string config_path = ament_index_cpp::get_package_share_directory("local_costmap") + 
                                 "/config/costmap_params.yaml";
        YAML::Node config = YAML::LoadFile(config_path);

        // Load parameters
        local_costmap_width_ = config["local_costmap"]["width"].as<double>();
        local_costmap_height_ = config["local_costmap"]["height"].as<double>();
        resolution_ = config["local_costmap"]["resolution"].as<double>();
        update_frequency_ = config["local_costmap"]["update_frequency"].as<double>();

        // Create timer for periodic updates
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/update_frequency_),
            std::bind(&LocalCostmapPublisher::updateCostmap, this));
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        global_map_ = *msg;
        have_map_ = true;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        current_pose_ = *msg;
        have_pose_ = true;
    }

    void updateCostmap()
    {
        if (!have_map_ || !have_pose_) {
            return;
        }

        nav_msgs::msg::OccupancyGrid local_map;
        local_map.header.stamp = this->now();
        local_map.header.frame_id = "map";
        local_map.info.resolution = resolution_;
        local_map.info.width = static_cast<unsigned int>(local_costmap_width_ / resolution_);
        local_map.info.height = static_cast<unsigned int>(local_costmap_height_ / resolution_);

        // Calculate origin for local map centered on robot
        local_map.info.origin.position.x = current_pose_.pose.position.x - local_costmap_width_/2;
        local_map.info.origin.position.y = current_pose_.pose.position.y - local_costmap_height_/2;
        
        // Initialize data vector
        local_map.data.resize(local_map.info.width * local_map.info.height);

        // Copy relevant cells from global map to local map
        for (unsigned int y = 0; y < local_map.info.height; ++y) {
            for (unsigned int x = 0; x < local_map.info.width; ++x) {
                // Calculate corresponding position in global map
                double world_x = local_map.info.origin.position.x + x * resolution_;
                double world_y = local_map.info.origin.position.y + y * resolution_;

                // Convert to global map cell coordinates
                int global_x = static_cast<int>((world_x - global_map_.info.origin.position.x) 
                                              / global_map_.info.resolution);
                int global_y = static_cast<int>((world_y - global_map_.info.origin.position.y) 
                                              / global_map_.info.resolution);

                // Check if point is within global map bounds
                if (global_x >= 0 && global_x < static_cast<int>(global_map_.info.width) &&
                    global_y >= 0 && global_y < static_cast<int>(global_map_.info.height)) {
                    local_map.data[y * local_map.info.width + x] = 
                        global_map_.data[global_y * global_map_.info.width + global_x];
                } else {
                    local_map.data[y * local_map.info.width + x] = -1; // Unknown
                }
            }
        }

        costmap_pub_->publish(local_map);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    nav_msgs::msg::OccupancyGrid global_map_;
    geometry_msgs::msg::PoseStamped current_pose_;
    bool have_map_ = false;
    bool have_pose_ = false;

    double local_costmap_width_;
    double local_costmap_height_;
    double resolution_;
    double update_frequency_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalCostmapPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}