#include <memory>
#include <queue>
#include <unordered_set>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "example_interfaces/srv/set_bool.hpp"


std::array<double, 2> convert_to_global_coords(std::array<int, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    double resolution = map->info.resolution;
    std::array<double, 2> origin = {map->info.origin.position.x, map->info.origin.position.y};
    std::array<double, 2> converted;
    for(int i = 0;i<2;i++) {
        converted[i] = origin[i] + (coords[i] * resolution);
    }
    return converted;
}

std::array<int, 2> convert_to_grid_coords(std::array<double, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    double resolution = map->info.resolution;
    std::array<double, 2> origin = {map->info.origin.position.x, map->info.origin.position.y};
    std::array<int, 2> converted;
    for(int i = 0;i<2;i++) {
        converted[i] = std::round((coords[i]-origin[i])/resolution);
    }
    return converted;
}



class MapToggleService : public rclcpp::Node
{
public:
    MapToggleService()
    : Node("intersection_clearing_service")
    {
        white_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local/temp", 10, std::bind(&MapToggleService::whiteMapCallback, this, std::placeholders::_1));

        yellow_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local/interp", 10, std::bind(&MapToggleService::yellowMapCallback, this, std::placeholders::_1));

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapToggleService::odomCallback, this, std::placeholders::_1));

        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/white/local", 10);

        service = this->create_service<example_interfaces::srv::SetBool>(
            "clear_intersection", std::bind(&MapToggleService::handleToggleRequest, this, std::placeholders::_1, std::placeholders::_2));

        white_map_recv = false;
        yellow_map_recv = false;
        odom_recv = false;
        running = false;
    
        RCLCPP_INFO(this->get_logger(), "intersection_clearing_service started.");
    }

private:
    // void conditional_publish() {
    //     if(!running) {
    //         map_pub->publish(*white_map_msg);
    //     } else {
    //         publish_map();
    //     }
    // }
    
    void whiteMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        white_map_recv = true;
        white_map_msg = msg;
        if(white_map_recv && odom_recv && yellow_map_recv) {
            publish_map(running);
        } else {
            publish_map(false);
        }
    }

    void yellowMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        yellow_map_recv = true;
        yellow_map_msg = msg;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_msg = msg;
        odom_recv = true;
    }

    // OPTIMIZED: Radial search with early termination
    bool get_nearest_point_bfs(const std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<int, 2>& dst) {
        // Early exit if source is already occupied  
        if (!map) return false;
        
        // Check bounds for source
        if (src[0] < 0 || src[0] >= map->info.width || 
            src[1] < 0 || src[1] >= map->info.height) {
            return false;
        }
        
        const int src_index = src[1] * map->info.width + src[0];
        if(map->data[src_index] > 0) {
            dst = src;
            return true;
        }
        
        const int width = map->info.width;
        const int height = map->info.height;
        const int max_radius = static_cast<int>(std::ceil(5.0 / map->info.resolution));
        
        // Radial search: check all points at each radius level
        for(int radius = 1; radius <= max_radius && running; ++radius) {
            // Check points on the perimeter of the circle
            for(int dx = -radius; dx <= radius; ++dx) {
                for(int dy = -radius; dy <= radius; ++dy) {
                    // Skip points not on the current radius circle
                    if(dx*dx + dy*dy != radius*radius && 
                       !(std::abs(dx) == radius || std::abs(dy) == radius)) {
                        continue;
                    }
                    
                    const int nx = src[0] + dx;
                    const int ny = src[1] + dy;
                    
                    // Bounds check
                    if(nx < 0 || nx >= width || ny < 0 || ny >= height) {
                        continue;
                    }
                    
                    // Check occupancy
                    const int index = ny * width + nx;
                    if(map->data[index] > 0) {
                        dst = {nx, ny};
                        return true;
                    }
                }
            }
        }
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
        double range, std::array<int, 2>& dst) {

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
                            dst = {white_x, white_y};
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


    nav_msgs::msg::OccupancyGrid clearRadiusAroundPoint(const nav_msgs::msg::OccupancyGrid::SharedPtr input_grid, 
                                                const std::array<int, 2>& point, 
                                                double radius) {
        // Create a copy of the input grid
        nav_msgs::msg::OccupancyGrid output_grid = *input_grid;
        
        // Get grid dimensions and resolution
        int width = input_grid->info.width;
        int height = input_grid->info.height;
        double resolution = input_grid->info.resolution;
        
        // Convert radius from meters to grid cells
        int radius_cells = static_cast<int>(std::ceil(radius / resolution));
        
        // Get the center point coordinates
        int center_x = point[0];
        int center_y = point[1];
        
        // Validate that the center point is within bounds
        if (center_x < 0 || center_x >= width || center_y < 0 || center_y >= height) {
            // Return original grid if center point is out of bounds
            return output_grid;
        }
        
        // Calculate bounding box for the search area
        int min_x = std::max(0, center_x - radius_cells);
        int max_x = std::min(width - 1, center_x + radius_cells);
        int min_y = std::max(0, center_y - radius_cells);
        int max_y = std::min(height - 1, center_y + radius_cells);
        
        // Iterate through the bounding box
        for (int y = min_y; y <= max_y; ++y) {
            for (int x = min_x; x <= max_x; ++x) {
                // Calculate distance from center point to current cell
                double dx = (x - center_x) * resolution;
                double dy = (y - center_y) * resolution;
                double distance = std::sqrt(dx * dx + dy * dy);
                
                // If within radius, set the cell value to 0
                if (distance <= radius) {
                    int index = y * width + x;
                    output_grid.data[index] = 0;
                }
            }
        }
        
        return output_grid;
    }


    void publish_map(bool condition)
    {
        /*
        Find nearest yellow lane point
        Find nearest white lane point from cluster
        Find nearest yellow lane point from white lane point
        Clear 2m grid radius of white points around 
        */
        if (!white_map_msg || !yellow_map_msg || !odom_msg) {
           RCLCPP_WARN(this->get_logger(), "Missing required map or odometry data");
            return;
        }
        std::array<double, 2> odom = {odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y};
        std::array<int, 2> odom_grid = convert_to_grid_coords(odom, white_map_msg);
        std::array<int, 2> nearest_yellow, nearest_white;
        bool status1 = get_nearest_point_bfs(odom_grid, yellow_map_msg, nearest_yellow);
        if(!status1) {
            map_pub->publish(*white_map_msg);
        }
        bool status2 = extendedBFS(yellow_map_msg, white_map_msg, nearest_yellow, 2.0, nearest_white);
        if(status2) {
            if(condition) {
                nav_msgs::msg::OccupancyGrid map = clearRadiusAroundPoint(white_map_msg, nearest_white, 2.85);
                map_pub->publish(map);
            } else {
                nav_msgs::msg::OccupancyGrid map = clearRadiusAroundPoint(white_map_msg, nearest_white, 1);
                map_pub->publish(map);
            }
        } else {
            map_pub->publish(*white_map_msg);
        }
    }


    void handleToggleRequest(
        const std::shared_ptr<example_interfaces::srv::SetBool::Request> request,
        std::shared_ptr<example_interfaces::srv::SetBool::Response> response)
    {
        running = request->data;

        publish_map(running);
    
        response->success = true;
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr white_map_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr yellow_map_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;

    rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr service;

    rclcpp::TimerBase::SharedPtr timer;

    nav_msgs::msg::OccupancyGrid::SharedPtr white_map_msg, yellow_map_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;

    bool running;

    bool white_map_recv;
    bool yellow_map_recv;
    bool odom_recv;

    bool recorded_pose;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapToggleService>());
    rclcpp::shutdown();
    return 0;
}