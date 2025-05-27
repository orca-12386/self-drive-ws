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

using std::placeholders::_1;

struct MapPose {
  int x, y;
  MapPose(int x_ = 0, int y_ = 0) : x(x_), y(y_) {}
};

struct WorldPose {
  double x, y;
  WorldPose(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
};

struct Map {
  int width, height;
  double resolution;
  WorldPose origin;
  std::vector<std::vector<int>> grid;
};

struct BotPose {
  WorldPose world_pose;
  MapPose map_pose;
  double pitch, roll, yaw;
};

class IntersectionDetectorNode : public rclcpp::Node {
private:
  /* rclcpp::TimerBase::SharedPtr main_timer; */
  rclcpp::Service<intersection_detector::srv::DetectIntersection>::SharedPtr
      detection_service;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_publisher;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscriber;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_publisher;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bool_publisher;

  bool have_map = false;
  bool have_odom = false;

  BotPose prev_pose;
  BotPose current_pose;
  Map current_map;

  MapPose getMapPoseFromWorldPose(const WorldPose &pose, const Map &map) {
    MapPose map_pose;
    map_pose.x = static_cast<int>((pose.x - map.origin.x) / map.resolution);
    map_pose.y = static_cast<int>((pose.y - map.origin.y) / map.resolution);
    return map_pose;
  }

  WorldPose getWorldPoseFromMapPose(const MapPose &pose, const Map &map) {
    WorldPose world_pose;
    world_pose.x = map.origin.x + (pose.x * map.resolution);
    world_pose.y = map.origin.y + (pose.y * map.resolution);
    return world_pose;
  }

  double distanceMap(const MapPose &a, const MapPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  double distanceWorld(const WorldPose &a, const WorldPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  MapPose findClosestMiddleLane(const MapPose &pose, const Map &map,
                                int radius) {
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

    std::queue<std::pair<MapPose, int>> q;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    std::unordered_set<int> visited;

    if (pose.x < 0 || pose.x >= map.width || pose.y < 0 ||
        pose.y >= map.height) {
      return MapPose(-1, -1);
    }

    q.push({pose, 0});
    visited.insert(hashFunc(pose));

    while (!q.empty()) {
      auto [current, distance] = q.front();
      q.pop();

      if (map.grid[current.y][current.x] == 100) {
        return current;
      }

      if (distance >= radius) {
        continue;
      }

      for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
            visited.find(hash) == visited.end()) {
          visited.insert(hash);
          q.push({neighbor, distance + 1});
        }
      }
    }

    return MapPose(-1, -1);
  }

  MapPose exploreMiddleLane(MapPose start, const Map &map) {
    const int dx[] = {0, 1, 0, -1};
    const int dy[] = {-1, 0, 1, 0};

    std::queue<MapPose> q;
    std::unordered_set<int> visited;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    q.push(start);
    visited.insert(hashFunc(start));

    MapPose last_cell = start;

    while (!q.empty()) {
      MapPose current = q.front();
      q.pop();

      last_cell = current;

      for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
            visited.find(hash) == visited.end() && map.grid[ny][nx] == 100) {
          visited.insert(hash);
          q.push(neighbor);
        }
      }
    }

    return last_cell;
  }

  /* void publishMarker(double x, double y, int id) { */
  /*   visualization_msgs::msg::Marker marker; */
  /*   marker.header.frame_id = "map"; */
  /*   marker.header.stamp = this->now(); */
  /*   marker.ns = "intersection_markers"; */
  /*   marker.id = id; */
  /*   marker.type = visualization_msgs::msg::Marker::SPHERE; */
  /*   marker.action = visualization_msgs::msg::Marker::ADD; */
  /**/
  /*   marker.pose.position.x = x; */
  /*   marker.pose.position.y = y; */
  /*   marker.pose.position.z = 0.0; */
  /**/
  /*   marker.pose.orientation.x = 0.0; */
  /*   marker.pose.orientation.y = 0.0; */
  /*   marker.pose.orientation.z = 0.0; */
  /*   marker.pose.orientation.w = 1.0; */
  /**/
  /*   marker.scale.x = 1.0; */
  /*   marker.scale.y = 1.0; */
  /*   marker.scale.z = 1.0; */
  /**/
  /*   marker.color.r = 0.0; */
  /*   marker.color.g = 0.0; */
  /*   marker.color.b = 1.0; */
  /*   marker.color.a = 1.0; */
  /**/
  /*   marker.lifetime = rclcpp::Duration(0, 0); */
  /**/
  /*   marker_publisher->publish(marker); */
  /* } */
  /**/
  /* void createArrowMarker(double curr_x, double curr_y, double prev_x, */
  /*                        double prev_y, int marker_id) { */
  /**/
  /*   visualization_msgs::msg::Marker marker; */
  /*   marker.header.frame_id = "map"; */
  /*   marker.header.stamp = rclcpp::Clock().now(); */
  /*   marker.ns = "arrow_marker"; */
  /*   marker.id = marker_id; */
  /*   marker.type = visualization_msgs::msg::Marker::ARROW; */
  /*   marker.action = visualization_msgs::msg::Marker::ADD; */
  /**/
  /*   double dx = curr_x - prev_x; */
  /*   double dy = curr_y - prev_y; */
  /*   double yaw = std::atan2(dy, dx); */
  /**/
  /*   marker.pose.position.x = prev_x; */
  /*   marker.pose.position.y = prev_y; */
  /*   marker.pose.position.z = 0.0; */
  /**/
  /*   tf2::Quaternion q; */
  /*   q.setRPY(0, 0, yaw); */
  /*   marker.pose.orientation.x = q.x(); */
  /*   marker.pose.orientation.y = q.y(); */
  /*   marker.pose.orientation.z = q.z(); */
  /*   marker.pose.orientation.w = q.w(); */
  /**/
  /*   marker.scale.x = 1.0; */
  /*   marker.scale.y = 0.1; */
  /*   marker.scale.z = 0.1; */
  /**/
  /*   marker.color.r = 1.0; */
  /*   marker.color.g = 0.0; */
  /*   marker.color.b = 0.0; */
  /*   marker.color.a = 1.0; */
  /**/
  /*   marker.lifetime = rclcpp::Duration::from_seconds(0); */
  /**/
  /*   marker_publisher->publish(marker); */
  /* } */

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
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

    have_map = true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
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

    if (!have_odom ||
        distanceWorld(prev_pose.world_pose, current_pose.world_pose) > 2) {
      prev_pose = current_pose;
    }

    if (have_map) {
      current_pose.map_pose =
          getMapPoseFromWorldPose(current_pose.world_pose, current_map);
    }

    /* RCLCPP_INFO(this->get_logger(), "Previous Pose: %f %f Current Pose: %f
     * %f", */
    /*             prev_pose.world_pose.x, prev_pose.world_pose.y, */
    /*             current_pose.world_pose.x, current_pose.world_pose.y); */

    have_odom = true;
  }

  void clearPixelsBehind(const MapPose &pose, double rounded_yaw) {
    int dx = 0, dy = 0;

    if (std::abs(rounded_yaw - 0) < 0.01 ||
        std::abs(rounded_yaw - 2 * M_PI) < 0.01) {
      dx = -1;
      dy = 0;
    } else if (std::abs(rounded_yaw - M_PI / 2) < 0.01) {
      dx = 0;
      dy = -1;
    } else if (std::abs(rounded_yaw - M_PI) < 0.01 ||
               std::abs(rounded_yaw + M_PI) < 0.01) {
      dx = 1;
      dy = 0;
    } else if (std::abs(rounded_yaw + M_PI / 2) < 0.01) {
      dx = 0;
      dy = 1;
    }

    if (dx != 0) {
      int startX = (dx > 0) ? pose.x + 1 : 0;
      int endX = (dx > 0) ? current_map.width - 1 : pose.x - 1;

      for (int x = startX; x <= endX; x++) {
        for (int y = 0; y < current_map.height; y++) {
          current_map.grid[y][x] = 0;
        }
      }
    } else if (dy != 0) {
      int startY = (dy > 0) ? pose.y + 1 : 0;
      int endY = (dy > 0) ? current_map.height - 1 : pose.y - 1;

      for (int y = startY; y <= endY; y++) {
        for (int x = 0; x < current_map.width; x++) {
          current_map.grid[y][x] = 0;
        }
      }
    }
  }

  /* void publishModifiedMap() { */
  /*   if (!have_map) */
  /*     return; */
  /**/
  /*   nav_msgs::msg::OccupancyGrid map_msg; */
  /**/
  /*   map_msg.header.stamp = this->now(); */
  /*   map_msg.header.frame_id = "map"; */
  /**/
  /*   map_msg.info.resolution = current_map.resolution; */
  /*   map_msg.info.width = current_map.width; */
  /*   map_msg.info.height = current_map.height; */
  /*   map_msg.info.origin.position.x = current_map.origin.x; */
  /*   map_msg.info.origin.position.y = current_map.origin.y; */
  /*   map_msg.info.origin.position.z = 0.0; */
  /*   map_msg.info.origin.orientation.w = 1.0; */
  /**/
  /*   map_msg.data.resize(current_map.width * current_map.height); */
  /*   for (int y = 0; y < current_map.height; y++) { */
  /*     for (int x = 0; x < current_map.width; x++) { */
  /*       int index = y * current_map.width + x; */
  /*       map_msg.data[index] = current_map.grid[y][x]; */
  /*     } */
  /*   } */
  /**/
  /*   map_publisher->publish(map_msg); */
  /* } */

  bool intersectionConfidence(const MapPose &scan_center, const Map &map,
                              int scan_radius = 50) {
    if (scan_center.x < 0 || scan_center.x >= map.width || scan_center.y < 0 ||
        scan_center.y >= map.height) {
      return false;
    }

    for (int y = std::max(0, scan_center.y - scan_radius);
         y <= std::min(map.height - 1, scan_center.y + scan_radius); y++) {
      for (int x = std::max(0, scan_center.x - scan_radius);
           x <= std::min(map.width - 1, scan_center.x + scan_radius); x++) {
        if (map.grid[y][x] == 100) {
          return false;
        }
      }
    }

    return true;
  }

  /* void detectIntersection() { */
  /*   if (have_map && have_odom) { */
  /*     double dx = current_pose.world_pose.x - prev_pose.world_pose.x; */
  /*     double dy = current_pose.world_pose.y - prev_pose.world_pose.y; */
  /*     double yaw; */
  /*     if (distanceWorld(current_pose.world_pose, prev_pose.world_pose) < 0.1)
   * { */
  /*       yaw = current_pose.yaw; */
  /*     } else { */
  /*       yaw = std::atan2(dy, dx); */
  /*     } */
  /**/
  /*     double rounded_yaw = std::round(yaw / (M_PI / 2)) * (M_PI / 2); */
  /**/
  /*     clearPixelsBehind(current_pose.map_pose, rounded_yaw); */
  /**/
  /*     MapPose mp = */
  /*         findClosestMiddleLane(current_pose.map_pose, current_map, 35); */
  /*     MapPose mpe = exploreMiddleLane(mp, current_map); */
  /*     WorldPose wpe = getWorldPoseFromMapPose(mpe, current_map); */
  /**/
  /*     wpe.x = wpe.x + 4.5 * cos(rounded_yaw); */
  /*     wpe.y = wpe.y + 4.5 * sin(rounded_yaw); */
  /**/
  /*     MapPose scan_center = getMapPoseFromWorldPose(wpe, current_map); */
  /**/
  /*     auto detection_msg = std_msgs::msg::Bool(); */
  /**/
  /*     if (intersectionConfidence(scan_center, current_map, 30)) */
  /*       detection_msg.data = true; */
  /*     else */
  /*       detection_msg.data = false; */
  /**/
  /*     bool_publisher->publish(detection_msg); */
  /**/
  /*   } */
  /* } */

  void handle_detection_request(
      const std::shared_ptr<
          intersection_detector::srv::DetectIntersection::Request>
          request,
      std::shared_ptr<intersection_detector::srv::DetectIntersection::Response>
          response) {
    (void)request; // Silence unused parameter warning

    if (have_map && have_odom) {
      // Existing detection logic from detectIntersection()
      double dx = current_pose.world_pose.x - prev_pose.world_pose.x;
      double dy = current_pose.world_pose.y - prev_pose.world_pose.y;
      double yaw;

      if (distanceWorld(current_pose.world_pose, prev_pose.world_pose) < 0.1) {
        yaw = current_pose.yaw;
      } else {
        yaw = std::atan2(dy, dx);
      }

      MapPose mp = findClosestMiddleLane(current_pose.map_pose, current_map, 300);
      WorldPose wpe = getWorldPoseFromMapPose(mp, current_map);
      wpe.x = wpe.x + 4.5 * cos(yaw);
      wpe.y = wpe.y + 4.5 * sin(yaw);
      MapPose scan_center = getMapPoseFromWorldPose(wpe, current_map);

      response->is_intersection =
          intersectionConfidence(scan_center, current_map, 20);
    } else {
      response->is_intersection = false;
    }
  }

public:
  IntersectionDetectorNode() : Node("intersection_detector_node") {
    map_subscriber = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map/yellow", 10,
        std::bind(&IntersectionDetectorNode::mapCallback, this, _1));
    odom_subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&IntersectionDetectorNode::odomCallback, this, _1));

    detection_service =
        create_service<intersection_detector::srv::DetectIntersection>(
            "/detection/intersection",
            std::bind(&IntersectionDetectorNode::handle_detection_request, this,
                      std::placeholders::_1, std::placeholders::_2));

    /* main_timer = this->create_wall_timer( */
    /*     std::chrono::milliseconds(100), */
    /*     std::bind(&IntersectionDetectorNode::detectIntersection, this)); */

    /* marker_publisher =
     * this->create_publisher<visualization_msgs::msg::Marker>( */
    /*     "/visualization_marker", 10); */
    /* map_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>( */
    /*     "/modified_map", 10); */
    /* bool_publisher = this->create_publisher<std_msgs::msg::Bool>( */
    /*     "detection/intersection", 10); */
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IntersectionDetectorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
