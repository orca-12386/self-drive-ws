#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <std_msgs/msg/float64.hpp>

class RobotPosePublisher : public rclcpp::Node {
public:
    RobotPosePublisher() : Node("robot_pose_publisher") {
        // Initialize subscribers
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white", 10, 
            std::bind(&RobotPosePublisher::mapCallback, this, std::placeholders::_1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&RobotPosePublisher::odomCallback, this, std::placeholders::_1));

        // Initialize publishers
        global_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "robot_pose_global", 10);
        grid_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "robot_pose_grid", 10);
        orientation_pub_ = create_publisher<std_msgs::msg::Float64>(
            "robot_orientation", 10);

        // Initialize TF buffer and listener
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

private:
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map_msg) {
        map_ = *map_msg;
        has_map_ = true;
    }

    std::pair<double, double> convertToMapCoords(const std::pair<double, double>& coord) {
        double origin_x = map_.info.origin.position.x;
        double origin_y = map_.info.origin.position.y;
        double resolution = map_.info.resolution;

        return std::make_pair(
            origin_x + coord.first * resolution,
            origin_y + coord.second * resolution
        );
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg) {
        if (!has_map_) {
            // RCLCPP_WARN(get_logger(), "No map received yet");
            return;
        }

        geometry_msgs::msg::PoseStamped pose_stamped;
        pose_stamped.header = odom_msg->header;
        pose_stamped.pose = odom_msg->pose.pose;

        try {
            // Transform from odom to map frame
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
            
            geometry_msgs::msg::PoseStamped robot_pose_map;
            tf2::doTransform(pose_stamped, robot_pose_map, transform);

            // Calculate grid coordinates
            int grid_x = static_cast<int>(
                (robot_pose_map.pose.position.x - map_.info.origin.position.x) / 
                map_.info.resolution);
            int grid_y = static_cast<int>(
                (robot_pose_map.pose.position.y - map_.info.origin.position.y) / 
                map_.info.resolution);

            // Publish global pose
            global_pose_pub_->publish(robot_pose_map);

            // Create and publish grid pose
            geometry_msgs::msg::PoseStamped grid_pose;
            grid_pose.header = robot_pose_map.header;
            grid_pose.pose.position.x = grid_x;
            grid_pose.pose.position.y = grid_y;
            grid_pose.pose.orientation = robot_pose_map.pose.orientation;
            grid_pose_pub_->publish(grid_pose);

            // Extract and publish yaw orientation
            auto orientation_msg = std_msgs::msg::Float64();  // Fixed: Correct instantiation
            tf2::Quaternion q(
                robot_pose_map.pose.orientation.x,
                robot_pose_map.pose.orientation.y,
                robot_pose_map.pose.orientation.z,
                robot_pose_map.pose.orientation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            orientation_msg.data = yaw;
            orientation_pub_->publish(orientation_msg);

        } catch (tf2::TransformException& ex) {
            RCLCPP_ERROR(get_logger(), "Transform failed: %s", ex.what());
            return;
        }
    }

    // Member variables
    nav_msgs::msg::OccupancyGrid map_;
    bool has_map_ = false;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr global_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr grid_pose_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr orientation_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPosePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}