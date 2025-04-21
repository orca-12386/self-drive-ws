#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

struct Point2D {
    double x;
    double y;
    
    Point2D(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
};

struct Cluster {
    std::vector<Point2D> points;
    Point2D centroid;
};

class MapClusterNode : public rclcpp::Node {
public:
    MapClusterNode() : Node("map_cluster_node") {
        // Subscribers
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local", 10, std::bind(&MapClusterNode::mapCallback, this, std::placeholders::_1));
        
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&MapClusterNode::odomCallback, this, std::placeholders::_1));
        
        // Publishers
        modified_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local/interp", 10);
        
        // Parameters
        this->declare_parameter("cluster_threshold", 0.5);  // Distance threshold for clustering
        this->declare_parameter("min_cluster_size", 10);    // Minimum points in a cluster
        this->declare_parameter("angle_threshold", 30.0);   // Maximum angle in degrees
        this->declare_parameter("distance_threshold", 10.0); // Maximum distance in meters
        this->declare_parameter("line_value", 90);          // Value for lines in occupancy grid (0-100)
        
        cluster_threshold_ = this->get_parameter("cluster_threshold").as_double();
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        angle_threshold_ = this->get_parameter("angle_threshold").as_double() * M_PI / 180.0;
        distance_threshold_ = this->get_parameter("distance_threshold").as_double();
        line_value_ = this->get_parameter("line_value").as_int();
        
        RCLCPP_INFO(this->get_logger(), "Map Cluster Node initialized");
    }

private:
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publishers
    // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr modified_map_pub_;
    
    // Robot position
    Point2D robot_pos_;
    
    // Parameters
    double cluster_threshold_;
    int min_cluster_size_;
    double angle_threshold_;
    double distance_threshold_;
    int line_value_;
    
    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        // Convert occupied points to world coordinates
        std::vector<Point2D> occupied_points;
        
        for (int y = 0; y < msg->info.height; ++y) {
            for (int x = 0; x < msg->info.width; ++x) {
                int index = y * msg->info.width + x;
                if (msg->data[index] > 50) {  // Occupied
                    double world_x = msg->info.origin.position.x + x * msg->info.resolution;
                    double world_y = msg->info.origin.position.y + y * msg->info.resolution;
                    occupied_points.emplace_back(world_x, world_y);
                }
            }
        }
        
        // Cluster occupied points
        std::vector<Cluster> clusters = clusterPoints(occupied_points);
        
        // Calculate centroids
        std::vector<Point2D> centroids;
        for (const auto& cluster : clusters) {
            centroids.push_back(cluster.centroid);
        }
        
        // Sort centroids by distance from robot
        std::sort(centroids.begin(), centroids.end(), [this](const Point2D& a, const Point2D& b) {
            double dist_a = std::hypot(a.x - robot_pos_.x, a.y - robot_pos_.y);
            double dist_b = std::hypot(b.x - robot_pos_.x, b.y - robot_pos_.y);
            return dist_a < dist_b;
        });
        
        // Draw lines between consecutive centroids with constraints
        drawLines(centroids);
        
        // Create and publish modified map
        createAndPublishModifiedMap(msg, centroids);
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_pos_.x = msg->pose.pose.position.x;
        robot_pos_.y = msg->pose.pose.position.y;
    }
    
    std::vector<Cluster> clusterPoints(const std::vector<Point2D>& points) {
        std::vector<Cluster> clusters;
        std::vector<bool> visited(points.size(), false);
        
        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;
            
            std::vector<Point2D> cluster_points;
            std::vector<size_t> stack;
            stack.push_back(i);
            visited[i] = true;
            
            // Simple region growing for clustering
            while (!stack.empty()) {
                size_t current = stack.back();
                stack.pop_back();
                cluster_points.push_back(points[current]);
                
                for (size_t j = 0; j < points.size(); ++j) {
                    if (!visited[j]) {
                        double dist = std::hypot(points[current].x - points[j].x, 
                                               points[current].y - points[j].y);
                        if (dist < cluster_threshold_) {
                            stack.push_back(j);
                            visited[j] = true;
                        }
                    }
                }
            }
            
            if (cluster_points.size() >= min_cluster_size_) {
                Cluster cluster;
                cluster.points = cluster_points;
                
                // Calculate centroid
                double sum_x = 0.0, sum_y = 0.0;
                for (const auto& p : cluster_points) {
                    sum_x += p.x;
                    sum_y += p.y;
                }
                cluster.centroid.x = sum_x / cluster_points.size();
                cluster.centroid.y = sum_y / cluster_points.size();
                
                clusters.push_back(cluster);
            }
        }
        
        return clusters;
    }
    
    void drawLines(const std::vector<Point2D>& centroids) {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;
        
        for (size_t i = 0; i < centroids.size() - 1; ++i) {
            const Point2D& current = centroids[i];
            const Point2D& next = centroids[i + 1];
            
            // Check distance constraint
            double distance_from_bot = std::hypot(next.x - robot_pos_.x, next.y - robot_pos_.y);
            if (distance_from_bot > distance_threshold_) {
                break;  // Stop if too far from robot (since they're sorted by distance)
            }
            
            // Calculate angle with previous line
            if (i > 0) {
                const Point2D& prev = centroids[i - 1];
                double angle = calculateAngle(prev, current, next);
                if (angle > angle_threshold_) {
                    continue;  // Skip if angle is too large
                }
            }
            
            // Create line marker
            visualization_msgs::msg::Marker line_marker;
            line_marker.header.frame_id = "map";
            line_marker.header.stamp = this->now();
            line_marker.ns = "cluster_lines";
            line_marker.id = id++;
            line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set line properties
            line_marker.scale.x = 0.1;  // Line width
            line_marker.color.r = 0.0;
            line_marker.color.g = 1.0;
            line_marker.color.b = 0.0;
            line_marker.color.a = 1.0;
            
            // Add points to line
            geometry_msgs::msg::Point p1, p2;
            p1.x = current.x;
            p1.y = current.y;
            p1.z = 0.0;
            p2.x = next.x;
            p2.y = next.y;
            p2.z = 0.0;
            
            line_marker.points.push_back(p1);
            line_marker.points.push_back(p2);
            
            marker_array.markers.push_back(line_marker);
        }
        
        // Clear old markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "map";
        clear_marker.header.stamp = this->now();
        clear_marker.ns = "cluster_lines";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.insert(marker_array.markers.begin(), clear_marker);
        
        // marker_pub_->publish(marker_array);
    }
    
    void createAndPublishModifiedMap(const nav_msgs::msg::OccupancyGrid::SharedPtr original_map,
                                     const std::vector<Point2D>& centroids) {
        // Create a copy of the original map
        auto modified_map = std::make_shared<nav_msgs::msg::OccupancyGrid>(*original_map);
        
        // Draw lines on the map
        for (size_t i = 0; i < centroids.size() - 1; ++i) {
            const Point2D& current = centroids[i];
            const Point2D& next = centroids[i + 1];
            
            // Check distance constraint
            double distance_from_bot = std::hypot(next.x - robot_pos_.x, next.y - robot_pos_.y);
            if (distance_from_bot > distance_threshold_) {
                break;  // Stop if too far from robot
            }
            
            // Check angle constraint
            if (i > 0) {
                const Point2D& prev = centroids[i - 1];
                double angle = calculateAngle(prev, current, next);
                if (angle > angle_threshold_) {
                    continue;  // Skip if angle is too large
                }
            }
            
            // Draw line using Bresenham's algorithm
            drawLineOnMap(modified_map, current, next);
        }
        
        // Publish the modified map
        modified_map_pub_->publish(*modified_map);
    }
    
    void drawLineOnMap(nav_msgs::msg::OccupancyGrid::SharedPtr map, 
                       const Point2D& start, const Point2D& end) {
        // Convert world coordinates to grid coordinates
        int x0 = static_cast<int>((start.x - map->info.origin.position.x) / map->info.resolution);
        int y0 = static_cast<int>((start.y - map->info.origin.position.y) / map->info.resolution);
        int x1 = static_cast<int>((end.x - map->info.origin.position.x) / map->info.resolution);
        int y1 = static_cast<int>((end.y - map->info.origin.position.y) / map->info.resolution);
        
        // Bresenham's line algorithm
        int dx = abs(x1 - x0);
        int dy = abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        
        while (true) {
            // Check boundaries
            if (x0 >= 0 && x0 < map->info.width && y0 >= 0 && y0 < map->info.height) {
                int index = y0 * map->info.width + x0;
                map->data[index] = line_value_;
            }
            
            if (x0 == x1 && y0 == y1) break;
            
            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
        }
    }
    
    double calculateAngle(const Point2D& p1, const Point2D& p2, const Point2D& p3) {
        // Calculate the angle between line segments p1-p2 and p2-p3
        double v1_x = p2.x - p1.x;
        double v1_y = p2.y - p1.y;
        double v2_x = p3.x - p2.x;
        double v2_y = p3.y - p2.y;
        
        double dot = v1_x * v2_x + v1_y * v2_y;
        double mag1 = std::hypot(v1_x, v1_y);
        double mag2 = std::hypot(v2_x, v2_y);
        
        if (mag1 < 1e-6 || mag2 < 1e-6) return 0.0;
        
        double cos_angle = dot / (mag1 * mag2);
        cos_angle = std::max(-1.0, std::min(1.0, cos_angle));
        
        return std::acos(cos_angle);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapClusterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}