#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/msg/marker.hpp"

#include "intersection_detector/srv/detect_intersection.hpp"

#include <cmath>
#include <memory>
#include <queue>
#include <vector>
#include <deque>
#include <unordered_set>
#include <set>


std::array<double, 2> convert_to_global_coords(std::array<int, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    double resolution = map->info.resolution;
    std::array<double, 2> origin = {map->info.origin.position.x, map->info.origin.position.y};
    std::array<double, 2> converted;
    for(int i = 0; i < 2; i++) {
        converted[i] = origin[i] + (coords[i] * resolution);
    }
    return converted;
}

std::array<int, 2> convert_to_grid_coords(std::array<double, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    double resolution = map->info.resolution;
    std::array<double, 2> origin = {map->info.origin.position.x, map->info.origin.position.y};
    std::array<int, 2> converted;
    for(int i = 0; i < 2; i++) {
        converted[i] = std::round((coords[i] - origin[i]) / resolution);
    }
    return converted;
}


class IntersectionDetectorNode : public rclcpp::Node
{
public:
    IntersectionDetectorNode() : 
    rclcpp::Node("intersection_detector_node")
    {
        RCLCPP_INFO(this->get_logger(), "intersection_detector_node started");
        this->initialise_data();
    };

private:
    void initialise_data() {
        yellow_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local/interp", 10, std::bind(&IntersectionDetectorNode::yellowMapCallback, this, std::placeholders::_1));

        white_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/map/white/local/near", 10, std::bind(&IntersectionDetectorNode::whiteMapCallback, this, std::placeholders::_1));
  
        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&IntersectionDetectorNode::odomCallback, this, std::placeholders::_1));

        detection_service =
        create_service<intersection_detector::srv::DetectIntersection>(
            "/detection/intersection",
            std::bind(&IntersectionDetectorNode::handle_detection_request, this,
                      std::placeholders::_1, std::placeholders::_2));

        yellow_map_recv = false;
        white_map_recv = false;
        odom_recv = false;
    }

    void yellowMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->yellow_map_msg = msg;
        yellow_map_recv = true;
    }

    void whiteMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->white_map_msg = msg;
        white_map_recv = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        odom_recv = true;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    bool get_nearest_point_bfs(const std::array<int, 2> src, 
        const nav_msgs::msg::OccupancyGrid::SharedPtr map, 
        std::array<int, 2>& dst) {
        
        RCLCPP_INFO(this->get_logger(), "Searching for nearest point from (%d, %d) in map %dx%d", 
                   src[0], src[1], map->info.width, map->info.height);
        
        std::queue<std::array<int, 2>> queue;
        std::set<std::array<int, 2>> visited;

        // First check if source is within bounds
        if (src[0] < 0 || src[0] >= static_cast<int>(map->info.width) || 
            src[1] < 0 || src[1] >= static_cast<int>(map->info.height)) {
            RCLCPP_WARN(this->get_logger(), "Source point (%d, %d) is outside map bounds", src[0], src[1]);
            return false;
        }

        queue.push(src);
        visited.insert(src);

        const int width = map->info.width;
        const int height = map->info.height;
        const int max_dist = static_cast<int>(std::ceil(10.0 / map->info.resolution)); // Increased search range

        const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

        int yellow_cells_found = 0;
        
        while (!queue.empty()) {
            auto current = queue.front();
            queue.pop();

            // Check if current point is occupied
            int idx = current[1] * width + current[0];
            if (idx >= 0 && idx < static_cast<int>(map->data.size())) {
                // Try different thresholds - occupancy grids can have different conventions
                if (map->data[idx] > 50 || map->data[idx] == 100) {
                    dst = current;
                    RCLCPP_INFO(this->get_logger(), "Found nearest yellow point at (%d, %d) with value %d", 
                               current[0], current[1], map->data[idx]);
                    return true;
                }
                if (map->data[idx] > 0) {
                    yellow_cells_found++;
                }
            }

            // Add neighbors within range
            for (int i = 0; i < 8; ++i) {
                int nx = current[0] + dx[i];
                int ny = current[1] + dy[i];

                if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;

                std::array<int, 2> neighbor = {nx, ny};
                if (visited.count(neighbor)) continue;

                // Check distance from source
                int dist_sq = (nx - src[0]) * (nx - src[0]) + (ny - src[1]) * (ny - src[1]);
                if (dist_sq <= max_dist * max_dist) {
                    visited.insert(neighbor);
                    queue.push(neighbor);
                }
            }
        }
        
        RCLCPP_WARN(this->get_logger(), "No yellow lane found within range. Cells with value > 0: %d", yellow_cells_found);
        return false;
    }

    struct Point {
        int x, y;
        Point(int x = 0, int y = 0) : x(x), y(y) {}
        
        bool operator==(const Point& other) const {
            return x == other.x && y == other.y;
        }
    };
    
    struct PointHash {
        std::size_t operator()(const Point& p) const {
            return std::hash<int>()(p.x) ^ (std::hash<int>()(p.y) << 1);
        }
    };
    
    bool extendedBFS(const nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map, 
        const nav_msgs::msg::OccupancyGrid::SharedPtr white_map, 
        const std::array<int, 2> src, 
        double range) {

        RCLCPP_INFO(this->get_logger(), "Starting extended BFS from yellow point (%d, %d) with range %.2fm", 
                   src[0], src[1], range);

        // Check if maps have similar characteristics
        if (std::abs(yellow_map->info.resolution - white_map->info.resolution) > 0.001) {
            RCLCPP_WARN(this->get_logger(), "Map resolutions differ: yellow=%.3f, white=%.3f", 
                       yellow_map->info.resolution, white_map->info.resolution);
        }

        std::queue<Point> queue;
        std::unordered_set<Point, PointHash> visited;

        Point source(src[0], src[1]);
        queue.push(source);
        visited.insert(source);

        const int width_yellow = yellow_map->info.width;
        const int height_yellow = yellow_map->info.height;
        const int width_white = white_map->info.width;
        const int height_white = white_map->info.height;

        const double resolution = yellow_map->info.resolution;
        const int range_cells = static_cast<int>(std::ceil(range / resolution));
        
        RCLCPP_INFO(this->get_logger(), "Range in cells: %d (resolution: %.3f)", range_cells, resolution);

        const int dx[] = {-1, -1, -1, 0, 0, 1, 1, 1};
        const int dy[] = {-1, 0, 1, -1, 1, -1, 0, 1};

        int yellow_points_checked = 0;
        int white_cells_found = 0;

        while (!queue.empty()) {
            Point current = queue.front();
            queue.pop();
            yellow_points_checked++;

            // Check white map around current yellow point
            // Simple approach: assume maps are aligned (same origin/resolution)
            for (int dy_check = -range_cells; dy_check <= range_cells; ++dy_check) {
                for (int dx_check = -range_cells; dx_check <= range_cells; ++dx_check) {
                    // Skip if outside circular range
                    if (dx_check * dx_check + dy_check * dy_check > range_cells * range_cells) continue;
                    
                    int white_x = current.x + dx_check;
                    int white_y = current.y + dy_check;
                    
                    // Check white map bounds
                    if (white_x < 0 || white_x >= width_white || 
                        white_y < 0 || white_y >= height_white) continue;
                    
                    // Check if white lane is occupied at this position
                    int white_idx = white_y * width_white + white_x;
                    if (white_idx >= 0 && white_idx < static_cast<int>(white_map->data.size())) {
                        if (white_map->data[white_idx] > 50 || white_map->data[white_idx] == 100) {
                            RCLCPP_INFO(this->get_logger(), "INTERSECTION FOUND! Yellow (%d,%d) near white (%d,%d)", 
                                       current.x, current.y, white_x, white_y);
                            return true;
                        }
                        if (white_map->data[white_idx] > 0) {
                            white_cells_found++;
                        }
                    }
                }
            }

            // Continue exploring along yellow lane - but limit the search
            if (yellow_points_checked > 1000) { // Prevent infinite loops
                RCLCPP_WARN(this->get_logger(), "BFS search limit reached");
                break;
            }

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx[i];
                int ny = current.y + dy[i];

                if (nx < 0 || nx >= width_yellow || ny < 0 || ny >= height_yellow) continue;

                Point neighbor(nx, ny);
                if (visited.count(neighbor)) continue;

                // Only queue if it's an occupied yellow lane cell
                int yellow_idx = ny * width_yellow + nx;
                if (yellow_idx >= 0 && yellow_idx < static_cast<int>(yellow_map->data.size())) {
                    if (yellow_map->data[yellow_idx] > 50 || yellow_map->data[yellow_idx] == 100) {
                        // Check distance from original source
                        int src_dx = nx - source.x;
                        int src_dy = ny - source.y;
                        int max_yellow_range = static_cast<int>(std::ceil(5.0 / resolution)); // 5m max along yellow
                        if (src_dx * src_dx + src_dy * src_dy <= max_yellow_range * max_yellow_range) {
                            visited.insert(neighbor);
                            queue.push(neighbor);
                        }
                    }
                }
            }
        }

        RCLCPP_INFO(this->get_logger(), "No intersection found. Yellow points checked: %d, White cells found: %d", 
                   yellow_points_checked, white_cells_found);
        return false;
    }

    bool check_intersection(const nav_msgs::msg::Odometry::SharedPtr odom, 
                           const nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map, 
                           const nav_msgs::msg::OccupancyGrid::SharedPtr white_map) {
        
        RCLCPP_INFO(this->get_logger(), "Checking intersection at robot position (%.2f, %.2f)", 
                   odom->pose.pose.position.x, odom->pose.pose.position.y);
        
        std::array<double, 2> odom_global = {odom->pose.pose.position.x, odom->pose.pose.position.y};
        std::array<int, 2> odom_grid = convert_to_grid_coords(odom_global, yellow_map);
        
        RCLCPP_INFO(this->get_logger(), "Robot grid position: (%d, %d)", odom_grid[0], odom_grid[1]);
        
        std::array<int, 2> nearest_yellow;
        bool status = get_nearest_point_bfs(odom_grid, yellow_map, nearest_yellow);
        if(!status) {
            log("Could not get nearest yellow lane");
            return false;
        }
        
        bool in_range = extendedBFS(yellow_map, white_map, nearest_yellow, 1.5);
        return in_range;
    }

    void handle_detection_request(
        const std::shared_ptr<
            intersection_detector::srv::DetectIntersection::Request>
            request,
        std::shared_ptr<intersection_detector::srv::DetectIntersection::Response>
            response) {
        (void)request;
  
        RCLCPP_INFO(this->get_logger(), "Detection request received. Maps received - Yellow: %s, White: %s, Odom: %s", 
                   yellow_map_recv ? "YES" : "NO", 
                   white_map_recv ? "YES" : "NO", 
                   odom_recv ? "YES" : "NO");
  
        if(white_map_recv && yellow_map_recv && odom_recv) {
            response->is_intersection = check_intersection(odometry_msg, yellow_map_msg, white_map_msg);
            RCLCPP_INFO(this->get_logger(), "Intersection detection result: %s", 
                       response->is_intersection ? "TRUE" : "FALSE");
        } else {
            response->is_intersection = false;
            RCLCPP_WARN(this->get_logger(), "Missing required data for intersection detection");
        }
    }  

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr white_map_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr yellow_map_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    bool white_map_recv;
    bool yellow_map_recv;
    bool odom_recv;
    nav_msgs::msg::OccupancyGrid::SharedPtr white_map_msg, yellow_map_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;
    rclcpp::Service<intersection_detector::srv::DetectIntersection>::SharedPtr detection_service;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntersectionDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}