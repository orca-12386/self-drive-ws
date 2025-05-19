#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <vector>
#include <cmath>

class ClusterGoalNode : public rclcpp::Node
{
public:
  ClusterGoalNode() : Node("cluster_goal_node")
  {
    // Declare parameters
    this->declare_parameter("odom_topic", "/odom/transformed");
    this->declare_parameter("map_topic", "/map/white");
    this->declare_parameter("goal_topic", "/goal_pose");
    this->declare_parameter("marker_topic", "/visualization_marker_array");
    this->declare_parameter("cluster_distance_threshold", 1.0);
    this->declare_parameter("forward_goal_distance", 1.5);
    this->declare_parameter("ahead_cluster_distance_", 1.0);

    // Get parameter values
    std::string odom_topic = this->get_parameter("odom_topic").as_string();
    std::string map_topic = this->get_parameter("map_topic").as_string();
    std::string goal_topic = this->get_parameter("goal_topic").as_string();
    std::string marker_topic = this->get_parameter("marker_topic").as_string();
    
    cluster_distance_threshold_ = this->get_parameter("cluster_distance_threshold").as_double();
    forward_goal_distance_ = this->get_parameter("forward_goal_distance").as_double();
    ahead_cluster_distance_ = this->get_parameter("ahead_cluster_distance_").as_double();
    // Create subscribers
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10, 
      std::bind(&ClusterGoalNode::odom_callback, this, std::placeholders::_1));
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic, 10, 
      std::bind(&ClusterGoalNode::map_callback, this, std::placeholders::_1));
    
    // Create publishers
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
    
    RCLCPP_INFO(this->get_logger(), "ClusterGoalNode initialized");
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  
  double cluster_distance_threshold_;
  double forward_goal_distance_;
  double ahead_cluster_distance_;
  
  struct Point {
    double x;
    double y;
  };
  
  struct Cluster {
    std::vector<Point> points;
    Point centroid;
    double distance_to_robot;
  };
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
    process_data();
  }
  
  void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    latest_map_ = msg;
    process_data();
  }
  
  void process_data()
  {
    if (!latest_odom_ || !latest_map_) {
      return;
    }
    
    // Get robot position
    double robot_x = latest_odom_->pose.pose.position.x;
    double robot_y = latest_odom_->pose.pose.position.y;
    
    // Get robot orientation (yaw)
    double qx = latest_odom_->pose.pose.orientation.x;
    double qy = latest_odom_->pose.pose.orientation.y;
    double qz = latest_odom_->pose.pose.orientation.z;
    double qw = latest_odom_->pose.pose.orientation.w;
    double yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    
    // Find obstacle points from the map
    std::vector<Point> obstacle_points = find_obstacles();
    
    // Cluster the obstacle points
    std::vector<Cluster> clusters = cluster_obstacles(obstacle_points, robot_x, robot_y);
    
    // Find the nearest cluster
    Cluster nearest_cluster;
    bool found_cluster = find_nearest_cluster(clusters, nearest_cluster);
    
    // Publish visualization markers
    publish_markers(clusters, found_cluster ? &nearest_cluster : nullptr);
    
    // Publish goal
    if (found_cluster && nearest_cluster.distance_to_robot >= cluster_distance_threshold_) {
      // If there's a nearby cluster, publish a goal near it
      publish_goal_near_cluster(nearest_cluster, robot_x, robot_y,yaw);
      RCLCPP_INFO(this->get_logger(), "Published goal near obstacle cluster at distance: %f", 
                 nearest_cluster.distance_to_robot);
    } else {
      // Otherwise, publish a goal directly ahead of the robot
      publish_forward_goal(robot_x, robot_y, yaw);
      RCLCPP_INFO(this->get_logger(), "Published forward goal");
    }
  }
  
  std::vector<Point> find_obstacles()
  {
    std::vector<Point> obstacles;
    
    if (!latest_map_) {
      return obstacles;
    }
    
    int width = latest_map_->info.width;
    int height = latest_map_->info.height;
    double resolution = latest_map_->info.resolution;
    double origin_x = latest_map_->info.origin.position.x;
    double origin_y = latest_map_->info.origin.position.y;
    
    // Iterate through the occupancy grid
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        int index = y * width + x;
        
        // Check if the cell is occupied (value = 100)
        if (latest_map_->data[index] == 100) {
          Point p;
          p.x = origin_x + (x + 0.5) * resolution;
          p.y = origin_y + (y + 0.5) * resolution;
          obstacles.push_back(p);
        }
      }
    }
    
    return obstacles;
  }
  
  std::vector<Cluster> cluster_obstacles(const std::vector<Point>& obstacles, double robot_x, double robot_y)
  {
    std::vector<Cluster> clusters;
    
    if (obstacles.empty()) {
      return clusters;
    }
    
    // A simple clustering algorithm based on distance
    double cluster_radius = 0.5;  // Points within this radius belong to the same cluster
    std::vector<bool> processed(obstacles.size(), false);
    
    for (size_t i = 0; i < obstacles.size(); ++i) {
      if (processed[i]) {
        continue;
      }
      
      Cluster new_cluster;
      new_cluster.points.push_back(obstacles[i]);
      processed[i] = true;
      
      // Find all points that belong to this cluster
      for (size_t j = 0; j < obstacles.size(); ++j) {
        if (processed[j]) {
          continue;
        }
        
        // Check if this point is close to any point in the cluster
        bool is_close = false;
        for (const auto& point : new_cluster.points) {
          double distance = std::hypot(obstacles[j].x - point.x, obstacles[j].y - point.y);
          if (distance <= cluster_radius) {
            is_close = true;
            break;
          }
        }
        
        if (is_close) {
          new_cluster.points.push_back(obstacles[j]);
          processed[j] = true;
        }
      }
      
      // Compute cluster centroid
      new_cluster.centroid.x = 0.0;
      new_cluster.centroid.y = 0.0;
      for (const auto& point : new_cluster.points) {
        new_cluster.centroid.x += point.x;
        new_cluster.centroid.y += point.y;
      }
      new_cluster.centroid.x /= new_cluster.points.size();
      new_cluster.centroid.y /= new_cluster.points.size();
      
      // Compute distance to robot
      new_cluster.distance_to_robot = std::hypot(
        new_cluster.centroid.x - robot_x, 
        new_cluster.centroid.y - robot_y
      );
      
      clusters.push_back(new_cluster);
    }
    
    return clusters;
  }
  
  bool find_nearest_cluster(const std::vector<Cluster>& clusters, Cluster& nearest)
  {
    if (clusters.empty()) {
      return false;
    }
    
    // Find the cluster with minimum distance to robot
    size_t min_index = 0;
    double min_distance = clusters[0].distance_to_robot;
    
    for (size_t i = 1; i < clusters.size(); ++i) {
      if (clusters[i].distance_to_robot < min_distance) {
        min_distance = clusters[i].distance_to_robot;
        min_index = i;
      }
    }
    
    nearest = clusters[min_index];
    return true;
  }
  
  void publish_markers(const std::vector<Cluster>& clusters, const Cluster* nearest_cluster)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // Publish all cluster centroids
    for (size_t i = 0; i < clusters.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = "map";
      marker.header.stamp = this->now();
      marker.ns = "cluster_centroids";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      
      marker.pose.position.x = clusters[i].centroid.x;
      marker.pose.position.y = clusters[i].centroid.y;
      marker.pose.position.z = 0.0;
      marker.pose.orientation.w = 1.0;
      
      marker.scale.x = 0.2;
      marker.scale.y = 0.2;
      marker.scale.z = 0.2;
      
      // Color depends on whether this is the nearest cluster
      if (nearest_cluster && clusters[i].centroid.x == nearest_cluster->centroid.x && 
          clusters[i].centroid.y == nearest_cluster->centroid.y) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
      } else {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
      }
      marker.color.a = 1.0;
      
      marker_array.markers.push_back(marker);
      
      // Also publish points for each cluster
      visualization_msgs::msg::Marker points_marker;
      points_marker.header.frame_id = "map";
      points_marker.header.stamp = this->now();
      points_marker.ns = "cluster_points";
      points_marker.id = static_cast<int>(i);
      points_marker.type = visualization_msgs::msg::Marker::POINTS;
      points_marker.action = visualization_msgs::msg::Marker::ADD;
      
      points_marker.pose.orientation.w = 1.0;
      
      points_marker.scale.x = 0.05;
      points_marker.scale.y = 0.05;
      
      // Same color as centroid
      points_marker.color = marker.color;
      points_marker.color.a = 0.7;
      
      for (const auto& point : clusters[i].points) {
        geometry_msgs::msg::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        points_marker.points.push_back(p);
      }
      
      marker_array.markers.push_back(points_marker);
    }
    
    marker_pub_->publish(marker_array);
  }
  void publish_goal_ahead_of_cluster(const Cluster& cluster, double robot_x, double robot_y, double robot_yaw)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    
    // Get direction from robot to cluster
    double dx = cluster.centroid.x - robot_x;
    double dy = cluster.centroid.y - robot_y;
    double distance = std::hypot(dx, dy);
    
    // Normalize the direction vector
    dx /= distance;
    dy /= distance;
    
    // Calculate position ahead of the cluster (in the same direction from robot to cluster)
    goal.pose.position.x = cluster.centroid.x + dx * ahead_cluster_distance_;
    goal.pose.position.y = cluster.centroid.y + dy * ahead_cluster_distance_;
    goal.pose.position.z = 0.0;
    
    // Use the same orientation as forward goals (robot's current orientation)
    double qz = std::sin(robot_yaw / 2.0);
    double qw = std::cos(robot_yaw / 2.0);
    
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = qz;
    goal.pose.orientation.w = qw;
    
    goal_pub_->publish(goal);
  }

  void publish_goal_near_cluster(const Cluster& cluster, double robot_x, double robot_y,double yaw)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    
    // Calculate a goal position that is near the cluster but not directly on it
    double dx = cluster.centroid.x - robot_x;
    double dy = cluster.centroid.y - robot_y;
    double distance = std::hypot(dx, dy);
    
    // Normalize the direction vector
    dx /= distance;
    dy /= distance;
    
    // Set goal position slightly before the cluster
    double offset = 0.5;  // Distance offset from cluster centroid
    // goal.pose.position.x = cluster.centroid.x - dx * offset;
    // goal.pose.position.y = 1.0 + cluster.centroid.y - dy * offset;
    // goal.pose.position.z = 0.0;
    

    goal.pose.position.x = cluster.centroid.x + dx * ahead_cluster_distance_;
    goal.pose.position.y = cluster.centroid.y + dy * ahead_cluster_distance_;
    goal.pose.position.z = 0.0;

    // Set orientation to face towards the cluster
    // double yaw = std::atan2(dy, dx);
    double qx = 0.0;
    double qy = 0.0;
    double qz = std::sin(yaw / 2.0);
    double qw = std::cos(yaw / 2.0);
    
    goal.pose.orientation.x = qx;
    goal.pose.orientation.y = qy;
    goal.pose.orientation.z = qz;
    goal.pose.orientation.w = qw;

    
    
    goal_pub_->publish(goal);
  }
  
  void publish_forward_goal(double robot_x, double robot_y, double yaw)
  {
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    
    // Calculate a forward position based on current position and orientation
    goal.pose.position.x = robot_x + forward_goal_distance_ * std::cos(yaw);
    goal.pose.position.y = robot_y + forward_goal_distance_ * std::sin(yaw);
    goal.pose.position.z = 0.0;
    
    // Set orientation (same as current robot orientation)
    double qz = std::sin(yaw / 2.0);
    double qw = std::cos(yaw / 2.0);
    
    goal.pose.orientation.x = 0.0;
    goal.pose.orientation.y = 0.0;
    goal.pose.orientation.z = qz;
    goal.pose.orientation.w = qw;
    
    goal_pub_->publish(goal);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClusterGoalNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
