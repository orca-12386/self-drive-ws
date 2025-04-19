#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class LaneCheckerNode : public rclcpp::Node
{
public:
    LaneCheckerNode()
    : Node("PullOut")
    {
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local", 10, std::bind(&LaneCheckerNode::map_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&LaneCheckerNode::odom_callback, this, std::placeholders::_1));

        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/occupied_cells_marker", 10);
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    }

private:
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
    geometry_msgs::msg::Pose latest_pose_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(),"Recieved Map");
        latest_map_ = msg;
        map_rec = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(!map_rec)
            return;

        latest_pose_ = msg->pose.pose;
        RCLCPP_INFO(this->get_logger(),"Search Started");
        search_and_publish();
    }

    void search_and_publish()
    {
        double search_length = 8.0; // meters
        double search_width = 1.0;  // meters

        double robot_x = latest_pose_.position.x;
        double robot_y = latest_pose_.position.y;

        tf2::Quaternion q(
            latest_pose_.orientation.x,
            latest_pose_.orientation.y,
            latest_pose_.orientation.z,
            latest_pose_.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        auto map = latest_map_;
        double resolution = map->info.resolution;
        double origin_x = map->info.origin.position.x;
        double origin_y = map->info.origin.position.y;
        int width = map->info.width;
        int height = map->info.height;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "occupied_cells";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = resolution;
        marker.scale.y = resolution;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        bool found  = false;
        double dist;
        for (double dx = 0; dx < search_length; dx += resolution)
        {
            for (double dy = -search_width / 2; dy <= search_width / 2; dy += resolution)
            {
                double local_x = dx;
                double local_y = dy;

                // Transform to global map frame
                double global_x = robot_x + local_x * cos(yaw) - local_y * sin(yaw);
                double global_y = robot_y + local_x * sin(yaw) + local_y * cos(yaw);

                int mx = (int)((global_x - origin_x) / resolution);
                int my = (int)((global_y - origin_y) / resolution);

                if (mx < 0 || my < 0 || mx >= width || my >= height)
                    continue;
               
                int index = my * width + mx;
                if (map->data[index] > 50)
                {
                    geometry_msgs::msg::Point p;
                    p.x = global_x;
                    p.y = global_y;
                    p.z = 0.05;
                    dist = std::hypot(global_x - robot_x, global_y - robot_y);
                    if (dist < 4)
                    {
                        found = true;
                    }

                    // Euclidean distance to the bot's center
 
                    geometry_msgs::msg::Twist vel;

                    vel.linear.x = 0.5;
                    if(dist < 4)
                        vel.linear.z = std::atan(1/dist);
                    else
                        vel.linear.z = 0;
                    
                    RCLCPP_INFO(this->get_logger(),"Publishing velocity commands:");
                    vel_pub_->publish(vel);
                    RCLCPP_INFO(this->get_logger(),
                        "Occupied Cell at [x: %.2f, y: %.2f] | Distance from bot: %.2f meters",
                        p.x, p.y, dist);

                    marker.points.push_back(p);
                }
            }
        }
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = 0.5; // constant linear velocity

        if (found)
        {
            double omega = std::atan(1.0 / dist);
            cmd_vel.angular.z = omega;
            RCLCPP_INFO(this->get_logger(), "Publishing Twist: linear=%.2f, angular=%.2f", cmd_vel.linear.x, cmd_vel.angular.z);
        }
        else
        {
            cmd_vel.angular.z = 0.0; // go straight
            RCLCPP_WARN(this->get_logger(), "No occupied lane found. Moving straight.");
        }
        vel_pub_->publish(cmd_vel);
        marker_pub_->publish(marker);
        }

    bool map_rec = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneCheckerNode>());
    rclcpp::shutdown();
    return 0;
}
