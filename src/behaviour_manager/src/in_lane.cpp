#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <visualization_msgs/msg/marker.hpp>


#include "utils.cpp"

class MapAndPoseSubscriberNode : public rclcpp::Node
{
public:
    MapAndPoseSubscriberNode()
        : Node("in_lane")
    {
        map_white_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local", 10,
            std::bind(&MapAndPoseSubscriberNode::whiteMapCallback, this, std::placeholders::_1));

        pose_sub = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10,
            std::bind(&MapAndPoseSubscriberNode::poseCallback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&MapAndPoseSubscriberNode::odomCallback, this, std::placeholders::_1));

        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>("/lane_markers", 10);
    }

private:
    void publishMarkers(const WorldPose& pt1, const WorldPose& pt2) {
        visualization_msgs::msg::Marker marker1;
        marker1.header.frame_id = "map";
        marker1.header.stamp = this->now();
        marker1.ns = "lane_points";
        marker1.id = 0;
        marker1.type = visualization_msgs::msg::Marker::SPHERE;
        marker1.action = visualization_msgs::msg::Marker::ADD;
        marker1.pose.position.x = pt1.x;
        marker1.pose.position.y = pt1.y;
        marker1.pose.position.z = 0.1;
        marker1.scale.x = 0.2;
        marker1.scale.y = 0.2;
        marker1.scale.z = 0.2;
        marker1.color.a = 1.0;
        marker1.color.r = 1.0;
        marker1.color.g = 0.0;
        marker1.color.b = 0.0;

        visualization_msgs::msg::Marker marker2 = marker1;
        marker2.id = 1;
        marker2.pose.position.x = pt2.x;
        marker2.pose.position.y = pt2.y;
        marker2.color.r = 0.0;
        marker2.color.g = 1.0;

        marker_pub->publish(marker1);
        marker_pub->publish(marker2);
    }



    void whiteMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        got_map = true;

        current_map.height = msg->info.height;
        current_map.width = msg->info.width;
        current_map.resolution = msg->info.resolution;
        current_map.origin.x = msg->info.origin.position.x;
        current_map.origin.y = msg->info.origin.position.y;

        current_map.grid.resize(current_map.height,
                                std::vector<int>(current_map.width, -1));

        for (int y = 0; y < current_map.height; y++) {
            for (int x = 0; x < current_map.width; x++) {
                int index = y * current_map.width + x;
                current_map.grid[y][x] = msg->data[index];
            }
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {

        current_pose.world_pose.x = msg->pose.pose.position.x;
        current_pose.world_pose.y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        current_pose.roll = roll;
        current_pose.pitch = pitch;
        current_pose.yaw = yaw;

        if (got_map) {
            current_pose.map_pose =
            Utils::getMapPoseFromWorldPose(current_pose.world_pose, current_map);
        }

        if (!got_odom || Utils::worldDistance(current_pose.world_pose, prev_pose.world_pose)) {
            prev_pose = current_pose;
        }

        got_odom = true;
    }

    void poseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        bool very_true = checkInLane(WorldPose(x, y), current_pose.world_pose);
    }

    bool checkInLane(WorldPose obs_pose, WorldPose bot_pose) {
        double theta;
        if (Utils::worldDistance(current_pose.world_pose, prev_pose.world_pose) <
            0.1) {
            theta = current_pose.yaw;
        } else {
            theta =
                Utils::getAngleRadians(prev_pose.world_pose, current_pose.world_pose);
        }

        Utils::removeMapBehindBot(current_map, current_pose.world_pose, theta, 20, 20);

        MapPose nearest_lane_mp = Utils::findClosestForValue(current_pose.map_pose, current_map, 300, 100);
        MapPose farthest_lane_mp = Utils::exploreLane(nearest_lane_mp, current_map, 100);

        WorldPose pt1 = Utils::getWorldPoseFromMapPose(nearest_lane_mp, current_map);
        WorldPose pt2 = Utils::getWorldPoseFromMapPose(farthest_lane_mp, current_map);

        publishMarkers(pt1, pt2);

        float dx = pt2.x - pt1.x;
        float dy = pt2.y - pt1.y;

        // Line equation: Ax + By + C = 0
        float A = dy;
        float B = -dx;
        float C = dx * pt1.y - dy * pt1.x;

        float d1 = std::abs(A * obs_pose.x + B * obs_pose.y + C) / std::sqrt(A * A + B * B);
        if (d1 < 0.5) {
            RCLCPP_INFO(this->get_logger(), "too close to the lane");
            return false;   
        }

        float d2 = std::abs(A * bot_pose.x + B * bot_pose.y + C) / std::sqrt(A * A + B * B);

        // Side of the line (sign of the equation without abs)
        float side1 = A * obs_pose.x + B * obs_pose.y + C;
        float side2 = A * bot_pose.x + B * bot_pose.y + C;

        bool close_enough = std::abs(d1 - d2) <= 2.0;
        bool same_side = (side1 * side2) >= 0;  // both same sign or zero

        RCLCPP_INFO(this->get_logger(), "d1: %.2f, d2: %.2f, same_side: %s", d1, d2, same_side ? "true" : "false");

        return close_enough && same_side;
    }



    // State flags
    bool got_map = false;
    bool got_odom = false;

    // Stored data
    Map current_map;
    BotPose current_pose;
    BotPose prev_pose;

    // Subscriptions
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_white_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_yellow_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapAndPoseSubscriberNode>());
    rclcpp::shutdown();
    return 0;
}
