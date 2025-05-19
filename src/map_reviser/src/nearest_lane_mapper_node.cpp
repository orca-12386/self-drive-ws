// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>


class NearestLaneMapperNode : public rclcpp::Node
{
public:
    NearestLaneMapperNode() : 
    rclcpp::Node("nearest_lane_mapper_node")
    {
        RCLCPP_INFO(this->get_logger(), "nearest_lane_mapper_node started");

        this->initialise_data();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NearestLaneMapperNode::timer_callback, this));
    };

private:
    void initialise_data() {
        std::string map_sub_topic("/map/white/local");
        std::string map_pub_topic("/map/white/local/near");

        map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_sub_topic, 10, std::bind(&NearestLaneMapperNode::mapCallback, this, std::placeholders::_1));

        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&NearestLaneMapperNode::odomCallback, this, std::placeholders::_1));

        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, 10);
        
        map_recv = false;
        odom_recv = false;
        
        nearest_map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        nearest_map_msg->header.frame_id = "map";
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map_msg = msg;
        map_recv = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        odom_recv = true;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

#include <array>
#include <queue>
#include <vector>
#include <unordered_set>
#include <cmath>

// Improved hash function for 2D coordinates
struct CoordHash {
    std::size_t operator()(const std::array<int, 2>& coords) const {
        return static_cast<std::size_t>(coords[0]) * 73856093 + 
               static_cast<std::size_t>(coords[1]) * 19349669;
    }
};

    // Get nearest non-zero point using BFS
    bool get_nearest_point_bfs(
        const std::array<int, 2>& src, 
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        std::array<int, 2>& dst
    ) {
        int MAX_SEARCH_DISTANCE = static_cast<int>(std::round(3.0 / map->info.resolution));
        
        // Directions: right, up, left, down
        static const std::array<std::array<int, 2>, 4> directions = {{
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}
        }};
        
        std::unordered_set<std::array<int, 2>, CoordHash> visited;
        std::queue<std::array<int, 2>> q;
        
        q.push(src);
        visited.insert(src);
        
        while (!q.empty()) {
            auto p = q.front();
            q.pop();
            
            // Check if this point is occupied (value > 0)
            const size_t index = p[1] * map->info.width + p[0];
            if (map->data[index] > 0) {
                dst = p;
                return true;
            }
            
            // Check if we've exceeded maximum search distance
            const double distance = std::hypot(p[0] - src[0], p[1] - src[1]);
            if (distance > MAX_SEARCH_DISTANCE) {
                continue;
            }
            
            // Check all four neighboring directions
            for (const auto& dir : directions) {
                std::array<int, 2> next = {p[0] + dir[0], p[1] + dir[1]};
                
                // Check map boundaries
                if (next[0] < 0 || next[0] >= map->info.width || 
                    next[1] < 0 || next[1] >= map->info.height) {
                    continue;
                }
                
                // Check if already visited
                if (visited.find(next) != visited.end()) {
                    continue;
                }
                
                q.push(next);
                visited.insert(next);
            }
        }
        
        return false;  // No valid point found
    }

   void get_connected_points_bfs(
    const std::array<int, 2>& src,
    const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
    std::vector<std::array<int, 2>>& connected,
    double max_skip_distance = 1.0  // Default skip distance in map units
    ) {
        connected.clear();
        
        const int width = map->info.width;
        const int height = map->info.height;
        
        // Convert skip distance from map units to grid cells
        const double skip_distance_cells = max_skip_distance / map->info.resolution;
        const double skip_distance_squared = skip_distance_cells * skip_distance_cells;
        
        // Find all non-zero points first
        std::vector<std::array<int, 2>> occupied_points;
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (map->data[y * width + x] > 0) {
                    occupied_points.push_back({x, y});
                }
            }
        }
        
        // BFS to find connected cluster
        std::unordered_set<std::array<int, 2>, CoordHash> visited;
        std::queue<std::array<int, 2>> q;
        
        // Start with source point
        q.push(src);
        visited.insert(src);
        
        while (!q.empty()) {
            auto current = q.front();
            q.pop();
            
            connected.push_back(current);
            
            // Find all nearby occupied points within skip distance
            for (const auto& point : occupied_points) {
                // Skip if already visited
                if (visited.find(point) != visited.end()) {
                    continue;
                }
                
                // Calculate squared distance
                const double dx = point[0] - current[0];
                const double dy = point[1] - current[1];
                const double dist_squared = dx*dx + dy*dy;
                
                // Check if within skip distance
                if (dist_squared <= skip_distance_squared) {
                    visited.insert(point);
                    q.push(point);
                }
            }
            
            // Safety check to prevent excessive memory usage
            if (connected.size() > 10000) {
                break;
            }
        }
    }

    void filter_and_publish_map(const nav_msgs::msg::Odometry::SharedPtr odom, const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        if(nearest_map_msg->data.size() != map->data.size()) {
            nearest_map_msg->data.resize(map->data.size(), 0);
        } else {
            std::fill(std::begin(nearest_map_msg->data), std::end(nearest_map_msg->data), 0);
        }
        nearest_map_msg->info.width = map->info.width;
        nearest_map_msg->info.height = map->info.height;
        nearest_map_msg->info.resolution = map->info.resolution;
        nearest_map_msg->info.origin.position.x = map->info.origin.position.x;
        nearest_map_msg->info.origin.position.y = map->info.origin.position.y;
        nearest_map_msg->info.origin.position.z = map->info.origin.position.z;
        nearest_map_msg->info.origin.orientation.w = map->info.origin.orientation.w;
        int grid_x = (odom->pose.pose.position.x - map->info.origin.position.x)/map->info.resolution;
        int grid_y = (odom->pose.pose.position.y - map->info.origin.position.y)/map->info.resolution; 
        std::array<int,2> source = {grid_x, grid_y};
        std::array<int,2> nearest_pt;
        // log("finding nearest");
        bool b = get_nearest_point_bfs(source, map, nearest_pt);
        if(!b) {
            map_pub->publish(*map);
            return;
        }
        // log("finding connected");
        std::vector<std::array<int,2>> connected;
        get_connected_points_bfs(nearest_pt, map, connected);
        // log("found connected");
        for(int i=0;i<connected.size();i++) {
            nearest_map_msg->data[connected[i][1]*nearest_map_msg->info.width + connected[i][0]] = 100;
        }
        nearest_map_msg->header.stamp = this->now();
        map_pub->publish(*nearest_map_msg);
        // log("published");
    }

    void timer_callback() {
        if(map_recv && odom_recv) {
            filter_and_publish_map(odometry_msg, map_msg);
        } else {
            log("Waiting for subscriptions");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    bool map_recv;
    bool odom_recv;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, nearest_map_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NearestLaneMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}