// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/srv/lane_follow_toggle.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>
#include <vector>
#include <unordered_set>
#include <Eigen/Geometry>

// Using std::array<double, 2> for all points
using Point2D = std::array<double, 2>;

// Optimized helper functions
inline double calculate_distance_sq(const Point2D& p1, const Point2D& p2) {
    double dx = p1[0] - p2[0];
    double dy = p1[1] - p2[1];
    return dx*dx + dy*dy;
}

inline double calculate_distance(const Point2D& p1, const Point2D& p2) {
    return std::sqrt(calculate_distance_sq(p1, p2));
}

inline double calculate_distance(const std::array<int, 2>& p1, const std::array<int, 2>& p2) {
    int dx = p1[0] - p2[0];
    int dy = p1[1] - p2[1];
    return std::sqrt(dx*dx + dy*dy);
}

// Fast angle calculation with pre-calculated constants
double calculate_angle_fast(const Point2D& candidate_goal, 
                           const Point2D& previous_goal, 
                           const Point2D& second_previous_goal) {
    Point2D v1 = {candidate_goal[0] - previous_goal[0], candidate_goal[1] - previous_goal[1]};
    Point2D v2 = {second_previous_goal[0] - previous_goal[0], second_previous_goal[1] - previous_goal[1]};
    
    // Quick zero check
    double v1_sq = v1[0]*v1[0] + v1[1]*v1[1];
    double v2_sq = v2[0]*v2[0] + v2[1]*v2[1];
    
    if (v1_sq < 1e-20 || v2_sq < 1e-20) return 0.0;
    
    // Fast angle calculation
    double dot = v1[0]*v2[0] + v1[1]*v2[1];
    double cross = std::abs(v1[0]*v2[1] - v1[1]*v2[0]);
    
    return std::atan2(cross, dot) * 57.2957795; // 180/π pre-calculated
}

// Simplified point at distance calculation
std::array<double, 3> get_point_at_distance(const std::array<double, 3>& point, 
    const std::array<double, 4>& quaternion, 
    double distance) {
    // Create quaternion using Eigen
    Eigen::Quaterniond quat(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
    quat.normalize();

    // Create forward vector [1, 0, 0]
    Eigen::Vector3d forward_vector(1.0, 0.0, 0.0);

    // Apply rotation to forward vector
    Eigen::Vector3d rotated_vector = quat * forward_vector;
    rotated_vector.normalize();
    rotated_vector *= distance;

    return {point[0] + rotated_vector[0], point[1] + rotated_vector[1], point[2] + rotated_vector[2]};
}

class LaneFollowerNode : public rclcpp::Node
{
public:
    LaneFollowerNode() : 
    rclcpp::Node("lane_follower_node")
    {
        RCLCPP_INFO(this->get_logger(), "lane_follower_node started");
        this->initialise_data();
        
        // Initialize with faster timer
        timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&LaneFollowerNode::timer_callback, this));
    };

private:
    // Pre-allocated data structures for performance
    std::vector<std::array<int, 2>> bfs_queue_storage;
    std::vector<bool> visited_grid;
    std::vector<std::array<int, 2>> directions_cache;
    
    // Cached map parameters
    double cached_resolution_inv = 0.0;
    std::array<double, 2> cached_origin = {0.0, 0.0};
    int cached_width = 0;
    int cached_height = 0;
    
    // ROS components
    rclcpp::Service<interfaces::srv::LaneFollowToggle>::SharedPtr toggle_srv;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map1_sub, map2_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    rclcpp::TimerBase::SharedPtr timer;
    
    // Data storage
    nav_msgs::msg::OccupancyGrid::SharedPtr map1_msg, map2_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg;
    
    // State variables
    bool map1_recv, map2_recv, odom_recv;
    bool running;
    bool start;
    std::vector<double> orientations;
    std::vector<Point2D> goals;
    double average_orientation;

    // Inline helper functions for speed
    inline uint64_t hash_coords_fast(int x, int y) const {
        return (static_cast<uint64_t>(x) << 32) | static_cast<uint64_t>(y);
    }
    
    inline bool is_valid_coord(int x, int y) const {
        return x >= 0 && x < cached_width && y >= 0 && y < cached_height;
    }
    
    inline int grid_index(int x, int y) const {
        return y * cached_width + x;
    }

    void update_map_cache(const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        cached_resolution_inv = 1.0 / map->info.resolution;
        cached_origin = {map->info.origin.position.x, map->info.origin.position.y};
        cached_width = map->info.width;
        cached_height = map->info.height;
        
        // Pre-allocate visited grid
        visited_grid.resize(cached_width * cached_height);
        
        // Cache direction vectors
        if (directions_cache.empty()) {
            directions_cache = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
            int skip_dist = static_cast<int>(std::round(3 * cached_resolution_inv));
            for(int i = 2; i <= skip_dist; i++) {
                directions_cache.push_back({i, 0});
                directions_cache.push_back({0, i});
                directions_cache.push_back({-i, 0});
                directions_cache.push_back({0, -i});
                directions_cache.push_back({i, i});
                directions_cache.push_back({-i, i});
                directions_cache.push_back({i, -i});
                directions_cache.push_back({-i, -i});
            }
        }
    }
    
    // Ultra-fast coordinate conversion with cached values
    inline std::array<double, 2> convert_to_global_coords_fast(int x, int y) const {
        return {cached_origin[0] + x / cached_resolution_inv, 
                cached_origin[1] + y / cached_resolution_inv};
    }
    
    inline std::array<int, 2> convert_to_grid_coords_fast(double x, double y) const {
        return {static_cast<int>(std::round((x - cached_origin[0]) * cached_resolution_inv)),
                static_cast<int>(std::round((y - cached_origin[1]) * cached_resolution_inv))};
    }

    // Legacy coordinate conversion functions for compatibility
    std::array<double, 2> convert_to_global_coords(std::array<int, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        return convert_to_global_coords_fast(coords[0], coords[1]);
    }

    std::array<int, 2> convert_to_grid_coords(std::array<double, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        return convert_to_grid_coords_fast(coords[0], coords[1]);
    }

    void initialise_data() {
        std::string map1_sub_topic("/map/white/local/near");
        std::string map2_sub_topic("/map/yellow/local/interp");

        map1_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map1_sub_topic, 10, std::bind(&LaneFollowerNode::map1Callback, this, std::placeholders::_1));
        
        map2_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map2_sub_topic, 10, std::bind(&LaneFollowerNode::map2Callback, this, std::placeholders::_1));
    
        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&LaneFollowerNode::odomCallback, this, std::placeholders::_1));

        goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        map1_recv = false;
        map2_recv = false;
        odom_recv = false;
        
        goal_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal_pose_msg->header.frame_id = "map";
        goal_pose_msg->header.stamp = this->now();
        goal_pose_msg->pose.position.z = 0.0;
        goal_pose_msg->pose.orientation.x = 0.0;
        goal_pose_msg->pose.orientation.y = 0.0;

        toggle_srv = this->create_service<interfaces::srv::LaneFollowToggle>("toggle_lane_follow", 
            std::bind(&LaneFollowerNode::toggle_lane_follow, this, std::placeholders::_1, std::placeholders::_2));
        
        running = true;
        start = true;
        
        // Pre-allocate containers
        goals.reserve(12);
        orientations.reserve(6);
        bfs_queue_storage.reserve(1000);
    }

    void toggle_lane_follow(const std::shared_ptr<interfaces::srv::LaneFollowToggle::Request> request, 
        std::shared_ptr<interfaces::srv::LaneFollowToggle::Response> response) {
        RCLCPP_INFO(this->get_logger(), "toggling lane follow to: %s", request->toggle ? "ON" : "OFF");

        if (request->toggle) {
            if (!running) {
                this->running = true;
                this->start = true;
                
                // Reset efficiently
                goals.clear();
                orientations.clear();
                
                // Cancel old timer
                if (timer) {
                    timer->cancel();
                    timer.reset();
                }
                
                // Create new timer with faster rate
                timer = this->create_wall_timer(
                    std::chrono::milliseconds(50), 
                    std::bind(&LaneFollowerNode::timer_callback, this));
                
                RCLCPP_INFO(this->get_logger(), "Lane following ON (fast mode)");
            } else {
                RCLCPP_INFO(this->get_logger(), "Lane following already running");
            }
        } else {
            if (running) {
                this->running = false; // Set first for immediate algorithm termination
                
                if (timer) {
                    timer->cancel();
                    timer.reset();
                }
                
                RCLCPP_INFO(this->get_logger(), "Lane following OFF");
            } else {
                RCLCPP_INFO(this->get_logger(), "Lane following already stopped");
            }
        }

        response->success = true;
    }

    void map1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map1_msg = msg;
        map1_recv = true;
    }

    void map2Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map2_msg = msg;
        map2_recv = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        odom_recv = true;
        
        // Fast yaw extraction
        double w = msg->pose.pose.orientation.w;
        double z = msg->pose.pose.orientation.z;
        double yaw = 2.0 * std::atan2(z, w);
        
        orientations.push_back(yaw);
        if (orientations.size() > 5) {
            orientations.erase(orientations.begin());
        }
        
        // Fast average calculation
        double sum = 0.0;
        for (double orient : orientations) {
            sum += orient;
        }
        average_orientation = sum / orientations.size();
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    double calculate_goal_angle(Point2D coords, const std::vector<Point2D>& goals) {
        if (goals.size() < 2) return 0.0;
        return calculate_angle_fast(coords, goals[goals.size()-1], goals[goals.size()-2]);
    }

    // Optimized BFS with early termination and limited search
    bool get_nearest_point_bfs_fast(int src_x, int src_y, const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<int, 2>& dst) {
        if (!running) return false;
        
        // Clear visited array efficiently
        std::fill(visited_grid.begin(), visited_grid.end(), false);
        
        // Use simple queue with pre-allocated storage
        bfs_queue_storage.clear();
        
        int queue_front = 0;
        bfs_queue_storage.push_back({src_x, src_y});
        visited_grid[grid_index(src_x, src_y)] = true;
        
        const int max_search_radius_sq = static_cast<int>(25 * cached_resolution_inv * cached_resolution_inv); // 5m radius squared
        
        while (queue_front < bfs_queue_storage.size()) {
            if ((queue_front & 15) == 0 && !running) return false; // Check every 16 iterations
            
            auto [x, y] = bfs_queue_storage[queue_front++];
            
            // Check if we found a lane point
            if (map->data[grid_index(x, y)] > 0) {
                dst = {x, y};
                return true;
            }
            
            // Distance check (squared to avoid sqrt)
            int dx = x - src_x;
            int dy = y - src_y;
            if (dx*dx + dy*dy > max_search_radius_sq) continue;
            
            // Add neighbors (only basic 4-connectivity for speed)
            for (int i = 0; i < 4; i++) {
                int nx = x + directions_cache[i][0];
                int ny = y + directions_cache[i][1];
                
                if (is_valid_coord(nx, ny)) {
                    int idx = grid_index(nx, ny);
                    if (!visited_grid[idx]) {
                        visited_grid[idx] = true;
                        bfs_queue_storage.push_back({nx, ny});
                    }
                }
            }
        }
        return false;
    }

    bool get_nearest_point_bfs(const std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<int, 2>& dst) {
        return get_nearest_point_bfs_fast(src[0], src[1], map, dst);
    }
    
    std::array<int, 2> get_best_point_hill_climb_fast(int src_x, int src_y, const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        if (!running || goals.size() < 2) return {src_x, src_y};
        
        std::array<int, 2> best_point = {src_x, src_y};
        double best_angle = -1.0;
        
        const int search_radius = static_cast<int>(7 * cached_resolution_inv); // 7m radius
        const int step_size = std::max(1, static_cast<int>(0.5 * cached_resolution_inv)); // 0.5m steps
        
        // Spiral search pattern for better coverage with fewer points
        for (int r = step_size; r <= search_radius; r += step_size) {
            if (!running) break;
            
            int num_samples = std::max(8, r / 2);
            for (int i = 0; i < num_samples; i++) {
                double angle = 2.0 * M_PI * i / num_samples;
                int x = src_x + static_cast<int>(r * std::cos(angle));
                int y = src_y + static_cast<int>(r * std::sin(angle));
                
                if (!is_valid_coord(x, y)) continue;
                
                if (map->data[grid_index(x, y)] > 0) {
                    auto global_coords = convert_to_global_coords_fast(x, y);
                    double goal_angle = calculate_goal_angle({global_coords[0], global_coords[1]}, goals);
                    
                    if (best_angle < 0 || goal_angle > best_angle) {
                        best_angle = goal_angle;
                        best_point = {x, y};
                    }
                }
            }
        }
        
        return best_point;
    }

    // Legacy hill climbing function for compatibility
    std::array<int, 2> get_best_point_hill_climb(std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        return get_best_point_hill_climb_fast(src[0], src[1], map);
    }

    // Streamlined publish_goal with early exits and caching
    void publish_goal(const nav_msgs::msg::Odometry::SharedPtr odom, 
                     const nav_msgs::msg::OccupancyGrid::SharedPtr map1, 
                     const nav_msgs::msg::OccupancyGrid::SharedPtr map2) {
        if (!running) return;
        
        // Update caches if needed (only when map changes)
        static int map1_seq = -1;
        if (map1->header.stamp.nanosec != map1_seq) {
            update_map_cache(map1);
            map1_seq = map1->header.stamp.nanosec;
        }
        
        // Fast coordinate conversion
        auto bot_grid = convert_to_grid_coords_fast(odom->pose.pose.position.x, odom->pose.pose.position.y);
        
        if (!running) return;
        
        // Handle initialization
        if (this->start) {
            Point2D bot_pos = {odom->pose.pose.position.x, odom->pose.pose.position.y};
            // Simple forward projection without expensive 3D math
            double yaw = average_orientation;
            Point2D forward_goal = {bot_pos[0] + 1.5 * std::cos(yaw), 
                                   bot_pos[1] + 1.5 * std::sin(yaw)};
            goals.clear();
            goals.push_back(bot_pos);
            goals.push_back(forward_goal);
            this->start = false;
        }
        
        if (!running) return;
        
        // Fast nearest point search
        std::array<int, 2> map1_src, map2_src;
        bool status1 = get_nearest_point_bfs_fast(bot_grid[0], bot_grid[1], map1, map1_src);
        
        if (!running || !status1) {
            if (!status1) log("Could not find nearest point in map1");
            return;
        }
        
        bool status2 = get_nearest_point_bfs_fast(bot_grid[0], bot_grid[1], map2, map2_src);
        
        if (!running || !status2) {
            if (!status2) log("Could not find nearest point in map2");
            return;
        }
        
        // Fast hill climbing
        auto parent1_grid = get_best_point_hill_climb_fast(map1_src[0], map1_src[1], map1);
        if (!running) return;
        
        auto parent2_grid = get_best_point_hill_climb_fast(map2_src[0], map2_src[1], map2);
        if (!running) return;
        
        // Convert and calculate goal
        auto parent1 = convert_to_global_coords_fast(parent1_grid[0], parent1_grid[1]);
        auto parent2 = convert_to_global_coords_fast(parent2_grid[0], parent2_grid[1]);
        Point2D goal = {(parent1[0] + parent2[0]) * 0.5, (parent1[1] + parent2[1]) * 0.5};
        
        // Quick goal validation and publishing
        bool should_update_goal = true;
        if (goals.size() >= 2) {
            double angle = calculate_goal_angle(goal, goals);
            should_update_goal = (angle > 90);
        }
        
        Point2D publish_goal_point = should_update_goal ? goal : goals.back();
        
        // Fast publish with cached orientation
        goal_pose_msg->header.stamp = this->now();
        goal_pose_msg->pose.position.x = publish_goal_point[0];
        goal_pose_msg->pose.position.y = publish_goal_point[1];
        goal_pose_msg->pose.orientation.z = std::sin(average_orientation * 0.5);
        goal_pose_msg->pose.orientation.w = std::cos(average_orientation * 0.5);
        goal_pub->publish(*goal_pose_msg);
        
        // Update goals list efficiently
        if (should_update_goal) {
            if (goals.empty() || calculate_distance(goal, goals.back()) >= 2.0) {
                goals.push_back(goal);
                if (goals.size() > 10) {
                    goals.erase(goals.begin()); // Remove oldest
                }
            }
        }
    }

    void timer_callback() {
        if (!running) return;
        
        if (map1_recv && map2_recv && odom_recv) {
            publish_goal(odometry_msg, map1_msg, map2_msg);
        } else {
            // Reduce logging frequency to avoid overhead
            static int counter = 0;
            if (++counter >= 100) { // Every ~5 seconds at 20Hz
                RCLCPP_INFO(this->get_logger(), "Waiting: map1=%s map2=%s odom=%s", 
                           map1_recv ? "✓" : "✗", map2_recv ? "✓" : "✗", odom_recv ? "✓" : "✗");
                counter = 0;
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}