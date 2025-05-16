#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <vector>
#include <queue>
#include <cmath>
#include "example_interfaces/srv/set_bool.hpp"


class YellowLineProcessor : public rclcpp::Node {
public:
  YellowLineProcessor() : Node("yellow_line_processor") {
    // Subscribers
    yellow_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map/yellow/local", 10, 
      std::bind(&YellowLineProcessor::yellowMapCallback, this, std::placeholders::_1));
    
    white_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map/white/local/near", 10, 
      std::bind(&YellowLineProcessor::whiteMapCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, 
      std::bind(&YellowLineProcessor::odomCallback, this, std::placeholders::_1));
    
    // Publisher
    interp_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/yellow/local/interp", 10);
    
    toggle_srv = this->create_service<example_interfaces::srv::SetBool>("toggle_lane_interpolation", std::bind(&YellowLineProcessor::toggle_interpolation, this, std::placeholders::_1, std::placeholders::_2));

    // Parameters
    min_cluster_size_ = this->declare_parameter<int>("min_cluster_size", 5);
    max_distance_between_centroids_ = this->declare_parameter<double>("max_distance_between_centroids", 5.0);
    max_angle_deviation_ = this->declare_parameter<double>("max_angle_deviation", 40.0 * M_PI / 180.0); // degrees to radians
    max_distance_to_white_ = this->declare_parameter<double>("max_distance_to_white", 7.0);
    min_distance_to_white_ = this->declare_parameter<double>("min_distance_to_white", 1.5);

    running = true;
  }

private:
  rclcpp::Service<example_interfaces::srv::SetBool>::SharedPtr toggle_srv;

  // Subscribers and publisher
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr yellow_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr white_map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr interp_map_pub_;
  
  // Store latest messages
  nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr white_map_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  
  // Parameters
  int min_cluster_size_;
  double max_distance_between_centroids_;
  double max_angle_deviation_;
  double max_distance_to_white_;
  double min_distance_to_white_;

  bool running;

  void toggle_interpolation(const std::shared_ptr<example_interfaces::srv::SetBool::Request> request, std::shared_ptr<example_interfaces::srv::SetBool::Response> response) {
    RCLCPP_INFO(this->get_logger(), "toggling interpolation");
    this->running = request->data;
    response->success = true;
  }

  void yellowMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    yellow_map_ = msg;
    if(running) {
      processYellowMap();
    } else {
      if(!yellow_map_) {
        RCLCPP_INFO(this->get_logger(), "Waiting for all required messages...");
        return;
      }
      interp_map_pub_->publish(*yellow_map_);
    }
  }
  
  void whiteMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    white_map_ = msg;
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    odom_ = msg;
  }
  
  void processYellowMap() {
    if (!yellow_map_ || !white_map_ || !odom_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for all required messages...");
      return;
    }
    
    // 1. Cluster filled parts in yellow map
    std::vector<std::vector<std::pair<int, int>>> clusters = clusterYellowMap();
    
    // 2. Filter clusters by size
    std::vector<std::vector<std::pair<int, int>>> valid_clusters;
    for (const auto& cluster : clusters) {
      if (cluster.size() >= static_cast<size_t>(min_cluster_size_)) {
        valid_clusters.push_back(cluster);
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "Found %zu clusters, %zu of which are valid by size",
    clusters.size(), valid_clusters.size());
  
    // Calculate centroids for valid clusters
    std::vector<std::pair<double, double>> centroids;
    for (const auto& cluster : valid_clusters) {
      centroids.push_back(calculateCentroid(cluster));
    }
    
    // Apply white lane proximity check early
    std::vector<std::pair<double, double>> filtered_centroids;
    for (const auto& centroid : centroids) {
      if (isNearWhiteLane(centroid)) {
        filtered_centroids.push_back(centroid);
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "After white lane proximity check: %zu of %zu centroids remain",
    filtered_centroids.size(), centroids.size());
    
    if (filtered_centroids.size() <= 1) {
      RCLCPP_INFO(this->get_logger(), "Not enough valid centroids after white lane proximity check.");
      interp_map_pub_->publish(*yellow_map_);
      return;
    }
    
    // 3. Get bot position in grid coordinates
    std::pair<double, double> bot_position = getBotPositionInGrid();
    
    // Find the nearest centroid to the bot
    std::vector<std::pair<double, double>> selected_centroids;
    int nearest_idx = findNearestCentroid(filtered_centroids, bot_position);
    selected_centroids.push_back(filtered_centroids[nearest_idx]);
    filtered_centroids.erase(filtered_centroids.begin() + nearest_idx);
    
    // 4. Iteratively get the nearest valid centroid
    while (!filtered_centroids.empty()) {
      std::pair<double, double> last_centroid = selected_centroids.back();
      nearest_idx = findNearestCentroid(filtered_centroids, last_centroid);
      std::pair<double, double> candidate = filtered_centroids[nearest_idx];
      
      bool is_valid = validateCentroid(candidate, selected_centroids);
      
      if (is_valid) {
        selected_centroids.push_back(candidate);
      }
      
      filtered_centroids.erase(filtered_centroids.begin() + nearest_idx);
    }
    
    RCLCPP_INFO(this->get_logger(), "Selected %zu centroids after validation", 
    selected_centroids.size());
  
    // 5. Interpolate a line and publish
    if (selected_centroids.size() > 1) {
      auto interpolated_map = interpolateLine(selected_centroids);
      interp_map_pub_->publish(interpolated_map);
    } else {
      RCLCPP_INFO(this->get_logger(), "No valid centroids after validation, publishing original map.");
      interp_map_pub_->publish(*yellow_map_);
    }
  }
  
  bool validateCentroid(const std::pair<double, double>& candidate, 
                      const std::vector<std::pair<double, double>>& selected_centroids) {
    if (selected_centroids.empty()) {
      return true;
    }
    
    const auto& last_centroid = selected_centroids.back();
    
    // 1. Check distance constraint
    double dx = candidate.first - last_centroid.first;
    double dy = candidate.second - last_centroid.second;
    double distance = std::sqrt(dx * dx + dy * dy) * yellow_map_->info.resolution;
    
    if (distance > max_distance_between_centroids_) {
      RCLCPP_INFO(this->get_logger(), "exceeds max distance");
      return false;
    }
    
    // 2. Check angle constraint if we have at least two centroids
    if (selected_centroids.size() >= 2) {
      const auto& second_last = selected_centroids[selected_centroids.size() - 2];
      
      double prev_dx = last_centroid.first - second_last.first;
      double prev_dy = last_centroid.second - second_last.second;
      
      double curr_angle = std::atan2(dy, dx);
      double prev_angle = std::atan2(prev_dy, prev_dx);
      
      double angle_diff = std::abs(curr_angle - prev_angle);
      // Normalize angle difference to [-π, π]
      if (angle_diff > M_PI) {
        angle_diff = 2 * M_PI - angle_diff;
      }
      
      if (angle_diff > max_angle_deviation_) {
        RCLCPP_INFO(this->get_logger(), "exceeds max angle");
        return false;
      }
    }
    
    return true;
  }

  std::vector<std::vector<std::pair<int, int>>> clusterYellowMap() {
    int width = yellow_map_->info.width;
    int height = yellow_map_->info.height;
    
    // Track visited cells
    std::vector<bool> visited(width * height, false);
    
    // Directions for BFS: right, left, down, up, and diagonals
    const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dy[] = {0, 0, 1, -1, 1, -1, -1, 1};
    
    std::vector<std::vector<std::pair<int, int>>> clusters;
    
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int idx = y * width + x;
        
        // Skip if cell is empty or already visited
        if (yellow_map_->data[idx] <= 0 || visited[idx]) {
          continue;
        }
        
        // Start a new cluster with BFS
        std::vector<std::pair<int, int>> cluster;
        std::queue<std::pair<int, int>> queue;
        
        queue.push({x, y});
        visited[idx] = true;
        
        while (!queue.empty()) {
          auto [cx, cy] = queue.front();
          queue.pop();
          cluster.push_back({cx, cy});
          
          for (int i = 0; i < 8; ++i) {
            int nx = cx + dx[i];
            int ny = cy + dy[i];
            
            if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
              int nidx = ny * width + nx;
              if (yellow_map_->data[nidx] > 0 && !visited[nidx]) {
                queue.push({nx, ny});
                visited[nidx] = true;
              }
            }
          }
        }
        
        clusters.push_back(cluster);
      }
    }
    
    return clusters;
  }
  
  std::pair<double, double> calculateCentroid(const std::vector<std::pair<int, int>>& cluster) {
    double sum_x = 0.0, sum_y = 0.0;
    for (const auto& [x, y] : cluster) {
      sum_x += x;
      sum_y += y;
    }
    return {sum_x / cluster.size(), sum_y / cluster.size()};
  }
  
  std::pair<double, double> getBotPositionInGrid() {
    // Convert odom position to grid coordinates
    double world_x = odom_->pose.pose.position.x;
    double world_y = odom_->pose.pose.position.y;
    
    // Transform to grid coordinates
    double grid_x = (world_x - yellow_map_->info.origin.position.x) / yellow_map_->info.resolution;
    double grid_y = (world_y - yellow_map_->info.origin.position.y) / yellow_map_->info.resolution;
    
    return {grid_x, grid_y};
  }
  
  int findNearestCentroid(const std::vector<std::pair<double, double>>& centroids, 
                          const std::pair<double, double>& point) {
    int nearest_idx = -1;
    double min_dist = std::numeric_limits<double>::max();
    
    for (size_t i = 0; i < centroids.size(); ++i) {
      double dx = centroids[i].first - point.first;
      double dy = centroids[i].second - point.second;
      double dist = std::sqrt(dx * dx + dy * dy);
      
      if (dist < min_dist) {
        min_dist = dist;
        nearest_idx = i;
      }
    }
    
    return nearest_idx;
  }
  
  
  bool isNearWhiteLane(const std::pair<double, double>& point) {
    int start_x = static_cast<int>(std::round(point.first));
    int start_y = static_cast<int>(std::round(point.second));
    int width = white_map_->info.width;
    int height = white_map_->info.height;
    
    // Check if starting point is within map bounds
    if (start_x < 0 || start_x >= width || start_y < 0 || start_y >= height) {
      RCLCPP_WARN(this->get_logger(), "Starting point (%d,%d) is outside map bounds", start_x, start_y);
      return false;
    }
    
    // BFS to find nearest white point
    std::queue<std::tuple<int, int, double>> queue;
    std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));
    
    queue.push({start_x, start_y, 0.0});
    visited[start_y][start_x] = true;
    
    // Directions for BFS
    const int dx[] = {1, -1, 0, 0, 1, -1, 1, -1};
    const int dy[] = {0, 0, 1, -1, 1, -1, -1, 1};
    
    while (!queue.empty()) {
      auto [x, y, distance] = queue.front();
      queue.pop();
      
      // Calculate actual Euclidean distance in grid cells
      double dist_cells = std::sqrt(std::pow(x - start_x, 2) + std::pow(y - start_y, 2));
      
      // Convert distance from grid cells to meters
      double dist_meters = dist_cells * white_map_->info.resolution;
      
      // If we've gone too far, skip this cell
      if (dist_meters > max_distance_to_white_) {
        continue;
      }
      
      // Check if current cell is white
      int idx = y * width + x;
      if (idx >= 0 && idx < static_cast<int>(white_map_->data.size()) && white_map_->data[idx] > 0) {
        RCLCPP_DEBUG(this->get_logger(), "Found white point at distance %.2f meters", dist_meters);
        if (dist_meters < min_distance_to_white_) {
          return false;
        }
        return true;
      }
      
      // Explore neighbors
      for (int i = 0; i < 8; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        
        // Check bounds
        if (nx >= 0 && nx < width && ny >= 0 && ny < height && !visited[ny][nx]) {
          visited[ny][nx] = true;
          
          // Use the accumulated distance for queue but don't use it for Euclidean distance calculation
          double step_dist = (i < 4) ? 1.0 : 1.414; // Straight vs diagonal step
          queue.push({nx, ny, distance + step_dist});
        }
      }
    }
    
    RCLCPP_DEBUG(this->get_logger(), "No white point found within %.2f meters", max_distance_to_white_);
    return false;
  }
  
  nav_msgs::msg::OccupancyGrid interpolateLine(const std::vector<std::pair<double, double>>& centroids) {
    // Create a new map based on the original
    nav_msgs::msg::OccupancyGrid result;
    result.header = yellow_map_->header;
    result.info = yellow_map_->info;
    
    // Initialize with same size as original map but all zeros
    result.data = yellow_map_->data;
    
    // If no centroids, return original map
    if (centroids.empty()) {
      RCLCPP_WARN(this->get_logger(), "No centroids to interpolate, returning original map.");
      return *yellow_map_;
    }
    
    // Draw lines between consecutive centroids
    for (size_t i = 1; i < centroids.size(); ++i) {
      drawLine(result, centroids[i-1], centroids[i]);
    }
    
    // Debug info
    RCLCPP_INFO(this->get_logger(), "Interpolated line through %zu centroids", centroids.size());
    
    return result;
  }
  
  void drawLine(nav_msgs::msg::OccupancyGrid& map, 
                const std::pair<double, double>& start, 
                const std::pair<double, double>& end) {
    int x0 = static_cast<int>(std::round(start.first));
    int y0 = static_cast<int>(std::round(start.second));
    int x1 = static_cast<int>(std::round(end.first));
    int y1 = static_cast<int>(std::round(end.second));
    
    int width = map.info.width;
    int height = map.info.height;
    
    // Debug output for line drawing
    RCLCPP_DEBUG(this->get_logger(), "Drawing line from (%d,%d) to (%d,%d)", x0, y0, x1, y1);
    
    // Bresenham's line algorithm
    bool steep = std::abs(y1 - y0) > std::abs(x1 - x0);
    if (steep) {
      std::swap(x0, y0);
      std::swap(x1, y1);
    }
    
    if (x0 > x1) {
      std::swap(x0, x1);
      std::swap(y0, y1);
    }
    
    int dx = x1 - x0;
    int dy = std::abs(y1 - y0);
    int error = dx / 2;
    int ystep = (y0 < y1) ? 1 : -1;
    int y = y0;
    
    for (int x = x0; x <= x1; ++x) {
      int map_x = steep ? y : x;
      int map_y = steep ? x : y;
      
      if (map_x >= 0 && map_x < width && map_y >= 0 && map_y < height) {
        int idx = map_y * width + map_x;
        if (idx >= 0 && idx < static_cast<int>(map.data.size())) {
          map.data[idx] = 100; // Mark as occupied (100%)
        }
      }
      
      error -= dy;
      if (error < 0) {
        y += ystep;
        error += dx;
      }
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<YellowLineProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}