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

    // Optimized BFS functions for faster performance

    // Use a simple 2D boolean array instead of unordered_set for visited tracking
    bool get_nearest_point_bfs(
        const std::array<int, 2>& src, 
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        std::array<int, 2>& dst
    ) {
        const int width = map->info.width;
        const int height = map->info.height;
        const int MAX_SEARCH_DISTANCE = static_cast<int>(std::round(3.0 / map->info.resolution));
        
        // Use 2D boolean array for much faster visited checking
        std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
        
        // Directions: right, down, left, up (reordered for better cache locality)
        static const std::array<std::array<int, 2>, 4> directions = {{
            {1, 0}, {0, 1}, {-1, 0}, {0, -1}
        }};
        
        std::queue<std::array<int, 2>> q;
        q.push(src);
        visited[src[1]][src[0]] = true;
        
        // Pre-calculate max distance squared to avoid sqrt
        const int max_dist_sq = MAX_SEARCH_DISTANCE * MAX_SEARCH_DISTANCE;
        
        while (!q.empty()) {
            auto p = q.front();
            q.pop();
            
            // Check if this point is occupied (value > 0)
            const size_t index = p[1] * width + p[0];
            if (map->data[index] > 0) {
                dst = p;
                return true;
            }
            
            // Check if we've exceeded maximum search distance (using squared distance)
            const int dx = p[0] - src[0];
            const int dy = p[1] - src[1];
            if (dx*dx + dy*dy > max_dist_sq) {
                continue;
            }
            
            // Check all four neighboring directions
            for (const auto& dir : directions) {
                const int next_x = p[0] + dir[0];
                const int next_y = p[1] + dir[1];
                
                // Check map boundaries and visited status in one go
                if (next_x >= 0 && next_x < width && 
                    next_y >= 0 && next_y < height && 
                    !visited[next_y][next_x]) {
                    
                    visited[next_y][next_x] = true;
                    q.push({next_x, next_y});
                }
            }
        }
        
        return false;  // No valid point found
    }

    void get_connected_points_bfs(
        const std::array<int, 2>& src,
        const nav_msgs::msg::OccupancyGrid::SharedPtr& map,
        std::vector<std::array<int, 2>>& connected,
        double max_skip_distance = 1.0
    ) {
        connected.clear();
        connected.reserve(1000); // Reserve space to avoid reallocations
        
        const int width = map->info.width;
        const int height = map->info.height;
        
        // Convert skip distance from map units to grid cells
        const double skip_distance_cells = max_skip_distance / map->info.resolution;
        const int skip_radius = static_cast<int>(std::ceil(skip_distance_cells));
        const double skip_distance_squared = skip_distance_cells * skip_distance_cells;
        
        // Use 2D boolean array for much faster visited checking
        std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
        
        // Create a spatial index for occupied points to speed up lookup
        std::vector<std::vector<std::vector<std::array<int, 2>>>> spatial_grid;
        const int grid_size = std::max(1, skip_radius * 2); // Grid cell size for spatial indexing
        const int grid_width = (width + grid_size - 1) / grid_size;
        const int grid_height = (height + grid_size - 1) / grid_size;
        spatial_grid.resize(grid_height, std::vector<std::vector<std::array<int, 2>>>(grid_width));
        
        // Populate spatial grid with occupied points
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (map->data[y * width + x] > 0) {
                    int grid_x = x / grid_size;
                    int grid_y = y / grid_size;
                    spatial_grid[grid_y][grid_x].push_back({x, y});
                }
            }
        }
        
        std::queue<std::array<int, 2>> q;
        q.push(src);
        visited[src[1]][src[0]] = true;
        
        while (!q.empty()) {
            auto current = q.front();
            q.pop();
            
            connected.push_back(current);
            
            // Determine which grid cells to search
            const int center_grid_x = current[0] / grid_size;
            const int center_grid_y = current[1] / grid_size;
            const int search_range = (skip_radius + grid_size - 1) / grid_size;
            
            const int min_grid_x = std::max(0, center_grid_x - search_range);
            const int max_grid_x = std::min(grid_width - 1, center_grid_x + search_range);
            const int min_grid_y = std::max(0, center_grid_y - search_range);
            const int max_grid_y = std::min(grid_height - 1, center_grid_y + search_range);
            
            // Search only in relevant grid cells
            for (int gy = min_grid_y; gy <= max_grid_y; ++gy) {
                for (int gx = min_grid_x; gx <= max_grid_x; ++gx) {
                    for (const auto& point : spatial_grid[gy][gx]) {
                        // Skip if already visited
                        if (visited[point[1]][point[0]]) {
                            continue;
                        }
                        
                        // Calculate squared distance
                        const int dx = point[0] - current[0];
                        const int dy = point[1] - current[1];
                        const double dist_squared = dx*dx + dy*dy;
                        
                        // Check if within skip distance
                        if (dist_squared <= skip_distance_squared) {
                            visited[point[1]][point[0]] = true;
                            q.push(point);
                        }
                    }
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