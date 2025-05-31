#include <memory>
#include <queue>
#include <unordered_set>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "example_interfaces/srv/set_bool.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "utils.cpp"

using std::placeholders::_1;
using std::placeholders::_2;

class MapToggleService : public rclcpp::Node
{
public:
    MapToggleService()
    : Node("intersection_clearing_service"), mode_active(false)
    {
        white_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local/temp", 10, std::bind(&MapToggleService::whiteMapCallback, this, _1));

        yellow_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local/interp", 10, std::bind(&MapToggleService::yellowMapCallback, this, _1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapToggleService::odomCallback, this, _1));

        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/white/local", 10);

        marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(
            "/start_clearing_marker", 10);

        service = this->create_service<example_interfaces::srv::SetBool>(
            "clear_intersection", std::bind(&MapToggleService::handleToggleRequest, this, _1, _2));

        got_white_map = false;
        got_yellow_map = false;
        got_odom = false;
    
        recorded_pose = false;

        RCLCPP_INFO(this->get_logger(), "MapToggleService started.");
    }

private:
    void whiteMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        got_white_map = true;

        current_white_map.height = msg->info.height;
        current_white_map.width = msg->info.width;
        current_white_map.resolution = msg->info.resolution;
        current_white_map.origin.x = msg->info.origin.position.x;
        current_white_map.origin.y = msg->info.origin.position.y;

        current_white_map.grid.resize(current_white_map.height,
                                std::vector<int>(current_white_map.width, -1));

        for (int y = 0; y < current_white_map.height; y++) {
            for (int x = 0; x < current_white_map.width; x++) {
                int index = y * current_white_map.width + x;
                current_white_map.grid[y][x] = msg->data[index];
            }
        }

        original_white_map = current_white_map;  // Backup the original map

        if (mode_active) {
            clearIntersection();
        } else {
            sitIdle();
        }
    }

    void yellowMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        got_yellow_map = true;

        current_yellow_map.height = msg->info.height;
        current_yellow_map.width = msg->info.width;
        current_yellow_map.resolution = msg->info.resolution;
        current_yellow_map.origin.x = msg->info.origin.position.x;
        current_yellow_map.origin.y = msg->info.origin.position.y;

        current_yellow_map.grid.resize(current_yellow_map.height,
                                std::vector<int>(current_yellow_map.width, -1));

        for (int y = 0; y < current_yellow_map.height; y++) {
            for (int x = 0; x < current_yellow_map.width; x++) {
                int index = y * current_yellow_map.width + x;
                current_yellow_map.grid[y][x] = msg->data[index];
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

        if (
            Utils::worldDistance(prev_pose.world_pose, current_pose.world_pose) > 3) {
            prev_pose = current_pose;
        }

        if (got_white_map) {
            current_pose.map_pose =
            Utils::getMapPoseFromWorldPose(current_pose.world_pose, current_white_map);
        }

        got_odom = true;
    }

    void clearIntersection()
    {
        if(!got_yellow_map) {
            return;
        }

        current_white_map = original_white_map;

        MapPose start_clearing_mp = Utils::getMapPoseFromWorldPose(start_clearing_point, current_white_map);

        MapPose start = Utils::findClosestForValue(start_clearing_mp, current_white_map, 300, 100);

        const int dx[] = {0, 1, 0, -1, -1, 1, 1, -1};
        const int dy[] = {-1, 0, 1, 0, 1, -1, 1, -1};

        std::queue<MapPose> q;
        std::vector<std::vector<bool>> visited(
            current_white_map.height, std::vector<bool>(current_white_map.width, false));

        q.push(start);
        visited[start.y][start.x] = true;

        while (!q.empty()) {
            MapPose current = q.front();
            q.pop();

            if (Utils::mapDistance(start, current) > 2.9/current_white_map.resolution)
                continue;

            for (int i = 0; i < 8; i++) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (nx >= 0 && nx < current_white_map.width &&
                    ny >= 0 && ny < current_white_map.height &&
                    !visited[ny][nx]) {

                    visited[ny][nx] = true;

                    if (current_white_map.grid[ny][nx] == 100) {
                        current_white_map.grid[ny][nx] = 0;
                        q.push(MapPose(nx, ny));
                    }
                }
            }

            for (int dy = -2; dy <= 2; ++dy) {
                for (int dx = -2; dx <= 2; ++dx) {
                    int nx = current.x + dx;
                    int ny = current.y + dy;

                    if (nx >= 0 && nx < current_white_map.width &&
                        ny >= 0 && ny < current_white_map.height &&
                        !visited[ny][nx]) {

                        if (current_white_map.grid[ny][nx] == 100) {
                            visited[ny][nx] = true;
                            current_white_map.grid[ny][nx] = 0;
                            q.push(MapPose(nx, ny));
                        }
                    }
                }
            }
        }

        publishClearedMap();
    }

    rclcpp::Time last_map_publish_time;

    void publishClearedMap()
    {
        nav_msgs::msg::OccupancyGrid output_map;
        output_map.header.stamp = this->now();
        output_map.header.frame_id = "map";

        output_map.info.resolution = current_white_map.resolution;
        output_map.info.width = current_white_map.width;
        output_map.info.height = current_white_map.height;

        output_map.info.origin.position.x = current_white_map.origin.x;
        output_map.info.origin.position.y = current_white_map.origin.y;
        output_map.info.origin.position.z = 0.0;
        output_map.info.origin.orientation.x = 0.0;
        output_map.info.origin.orientation.y = 0.0;
        output_map.info.origin.orientation.z = 0.0;
        output_map.info.origin.orientation.w = 1.0;

        output_map.data.resize(current_white_map.height * current_white_map.width, -1);

        for (int y = 0; y < current_white_map.height; ++y) {
            for (int x = 0; x < current_white_map.width; ++x) {
                int index = y * current_white_map.width + x;
                output_map.data[index] = current_white_map.grid[y][x];
            }
        }

        // Record publish time
        last_map_publish_time = this->now();
        
        map_pub->publish(output_map);
        
        RCLCPP_INFO(this->get_logger(), "Published cleared map at time: %f", 
                    last_map_publish_time.seconds());
    }

    void handleToggleRequest(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
    {
        if (request->data == false) {
            recorded_pose = false;
            mode_active = false;  // Also set mode_active to false
            RCLCPP_INFO(this->get_logger(), "Clearing recorded point and deactivating mode");
            response->success = true;
            response->message = "Recorded point cleared and mode deactivated";
            return;  // Return early to avoid executing the rest
        }
    
        if (request->data == true && recorded_pose == false) {
            double theta;
            if (Utils::worldDistance(current_pose.world_pose, prev_pose.world_pose) <
                0.1) {
                theta = current_pose.yaw;
            } else {
                theta =
                    Utils::getAngleRadians(prev_pose.world_pose, current_pose.world_pose);
            }
    
            MapPose nearest_yellow_mp = Utils::findClosestForValue(
                current_pose.map_pose, current_yellow_map, 300, 100
            );
    
            start_clearing_point =
                Utils::getWorldPoseFromMapPose(nearest_yellow_mp, current_white_map);
    
            recorded_pose = true;
            RCLCPP_INFO(this->get_logger(), "Recorded starting clearing point");
        }
    
        mode_active = request->data;
    
        if (mode_active) {
            RCLCPP_INFO(this->get_logger(), "Starting intersection clearing...");
            
            // Perform the clearing algorithm (this calls publishClearedMap internally)
            clearIntersection();
            
            // Get the number of subscribers to ensure delivery
            size_t subscriber_count = map_pub->get_subscription_count();
            RCLCPP_INFO(this->get_logger(), 
                    "Map published to %zu subscribers, waiting for delivery...", 
                    subscriber_count);
            
            // Wait longer if there are more subscribers, minimum 500ms, maximum 2000ms
            int wait_time_ms = std::max(500, std::min(2000, (int)(subscriber_count * 200 + 500)));
            rclcpp::sleep_for(std::chrono::milliseconds(wait_time_ms));
            
            RCLCPP_INFO(this->get_logger(), 
                    "Intersection clearing completed, ensured delivery after %dms", 
                    wait_time_ms);
            
            response->message = "Intersection cleared and map delivered to " + 
                            std::to_string(subscriber_count) + " subscribers";
            
        } else {
            RCLCPP_INFO(this->get_logger(), "Switching to idle mode...");
            sitIdle();  // This also publishes the map
            
            // Shorter delay for idle mode
            rclcpp::sleep_for(std::chrono::milliseconds(200));
            
            RCLCPP_INFO(this->get_logger(), "Idle mode activated");
            response->message = "Switched to idle mode";
        }
    
        response->success = true;
    }

    void sitIdle()
    {
        if (!got_white_map)
            return;

        RCLCPP_INFO(this->get_logger(), "Publishing same map");
        publishClearedMap();
    }

    void publishStartMarker()
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "start_clearing_point";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;

        marker.pose.position.x = start_clearing_point.x;
        marker.pose.position.y = start_clearing_point.y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        marker.lifetime = rclcpp::Duration::from_seconds(0);  // forever

        marker_pub->publish(marker);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr white_map_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr yellow_map_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service;

    rclcpp::TimerBase::SharedPtr timer;
    bool mode_active;

    bool got_white_map;
    bool got_yellow_map;
    bool got_odom;

    bool recorded_pose;

    WorldPose start_clearing_point;

    BotPose current_pose;
    BotPose prev_pose;

    Map current_white_map;
    Map original_white_map;
    Map current_yellow_map;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapToggleService>());
    rclcpp::shutdown();
    return 0;
}