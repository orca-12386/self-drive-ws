#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include "interfaces/action/goal_action.hpp"

#include "utils.cpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class FindIntersectionActionServer : public rclcpp::Node
{
public:
  using FindIntersection = interfaces::action::GoalAction;
  using GoalHandleFindIntersection = rclcpp_action::ServerGoalHandle<FindIntersection>;

  explicit FindIntersectionActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("find_intersection_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<FindIntersection>(
      this,
      "stop_intersection",
      std::bind(&FindIntersectionActionServer::handle_goal, this, _1, _2),
      std::bind(&FindIntersectionActionServer::handle_cancel, this, _1),
      std::bind(&FindIntersectionActionServer::handle_accepted, this, _1));

    auto map_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Subscribe to the first occupancy grid map
    map1_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map/white/local/near", map_qos,
      std::bind(&FindIntersectionActionServer::map1_callback, this, _1));

    // Subscribe to the second occupancy grid map
    map2_subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map/yellow/local/interp", map_qos,
      std::bind(&FindIntersectionActionServer::map2_callback, this, _1));

    // Subscribe to the odometry topic
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&FindIntersectionActionServer::odom_callback, this, _1));

    // Create publisher for the goal
    goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

    RCLCPP_INFO(this->get_logger(), "Find Intersection Action Server has been initialized");
  }

private:
  rclcpp_action::Server<FindIntersection>::SharedPtr action_server_;
  
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map1_subscription_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map2_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  nav_msgs::msg::OccupancyGrid::SharedPtr map1_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map2_;
  nav_msgs::msg::Odometry::SharedPtr odom_;

  Map current_map_yellow;
  Map current_map_white;
  BotPose current_pose;
  BotPose prev_pose;
  
  geometry_msgs::msg::PoseStamped current_goal_;
  bool goal_reached_ = false;
  const double goal_threshold_ = 0.6;  

  void map1_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received map1 update");
    map1_ = msg;

    current_map_white.height = msg->info.height;
    current_map_white.width = msg->info.width;
    current_map_white.resolution = msg->info.resolution;
    current_map_white.origin.x = msg->info.origin.position.x;
    current_map_white.origin.y = msg->info.origin.position.y;

    current_map_white.grid.resize(current_map_white.height,
                            std::vector<int>(current_map_white.width, -1));

    for (int y = 0; y < current_map_white.height; y++) {
      for (int x = 0; x < current_map_white.width; x++) {
        int index = y * current_map_white.width + x;
        current_map_white.grid[y][x] = msg->data[index];
      }
    }
  }

  void map2_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received map2 update");
    map2_ = msg;

    current_map_yellow.height = msg->info.height;
    current_map_yellow.width = msg->info.width;
    current_map_yellow.resolution = msg->info.resolution;
    current_map_yellow.origin.x = msg->info.origin.position.x;
    current_map_yellow.origin.y = msg->info.origin.position.y;

    current_map_yellow.grid.resize(current_map_yellow.height,
                            std::vector<int>(current_map_yellow.width, -1));

    for (int y = 0; y < current_map_yellow.height; y++) {
      for (int x = 0; x < current_map_yellow.width; x++) {
        int index = y * current_map_yellow.width + x;
        current_map_yellow.grid[y][x] = msg->data[index];
      }
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received odometry update");
    odom_ = msg;

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

    if (!odom_ ||
        Utils::worldDistance(prev_pose.world_pose, current_pose.world_pose) > 2) {
      prev_pose = current_pose;
    }

    if (map1_) {
      current_pose.map_pose =
          Utils::getMapPoseFromWorldPose(current_pose.world_pose, current_map_white);
    }
  }

  geometry_msgs::msg::PoseStamped find_goal()
  {
    if (!map1_ || !map2_ || !odom_) {
      RCLCPP_WARN(this->get_logger(), "Cannot find goal: missing map or odometry data");
      return geometry_msgs::msg::PoseStamped();
    }

    double theta;
    if (Utils::worldDistance(current_pose.world_pose, prev_pose.world_pose) <
        0.1) {
      theta = current_pose.yaw;
    } else {
      theta =
          Utils::getAngleRadians(prev_pose.world_pose, current_pose.world_pose);
    }

    Utils::removeMapBehindBot(
        current_map_yellow, current_pose.world_pose, theta, 20, 20
    );

    MapPose nearest_yellow_mp = Utils::findClosestForValue(
        current_pose.map_pose, current_map_yellow, 300, 100
    );

    MapPose farthest_yellow_mp =
        Utils::exploreLane(nearest_yellow_mp, current_map_yellow);

    MapPose first_white_mp = Utils::findClosestForValue(
        farthest_yellow_mp, current_map_white, 300, 100
    );

    if(first_white_mp.x == -1 && first_white_mp.y == -1) {
      return geometry_msgs::msg::PoseStamped();
    }

    std::queue<MapPose> q;
    std::set<std::pair<int, int>> visited;

    q.push(first_white_mp);
    visited.insert({first_white_mp.x, first_white_mp.y});

    MapPose white_mp = first_white_mp;

    int max_distance = 50;  // pixels

    while (!q.empty()) {
      MapPose current = q.front();
      q.pop();

      white_mp = current;  // last valid pixel

      // Search a 5x5 neighborhood
      for (int dx = -10; dx <= 10; ++dx) {
        for (int dy = -10; dy <= 10; ++dy) {
          int nx = current.x + dx;
          int ny = current.y + dy;

          if (visited.count({nx, ny})) continue;

          // Only consider pixels within map bounds
          if (nx < 0 || ny < 0 || nx >= static_cast<int>(current_map_white.width) ||
              ny >= static_cast<int>(current_map_white.height)) {
            continue;
          }

          // Limit search radius from the starting point
          if (std::abs(nx - first_white_mp.x) > max_distance ||
              std::abs(ny - first_white_mp.y) > max_distance) {
            continue;
          }

          if (current_map_white.grid[ny][nx] == 100) {
            q.push({nx, ny});
            visited.insert({nx, ny});
          }
        }
      }
    }


    RCLCPP_INFO(this->get_logger(), "%d %d %d %d", first_white_mp.x, first_white_mp.y, white_mp.x, white_mp.y);

    WorldPose white_wp =
        Utils::getWorldPoseFromMapPose(white_mp, current_map_white);

    double goal_theta = Utils::getAngleRadians(current_pose.world_pose, white_wp);

    double goal_x = white_wp.x - 0.715 * cos(goal_theta);
    double goal_y = white_wp.y - 0.715 * sin(goal_theta);

    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";
    goal.header.stamp = this->now();
    
    goal.pose.position.x = goal_x;
    goal.pose.position.y = goal_y;
    goal.pose.position.z = 0.0;
    if (odom_) {
      goal.pose.orientation = odom_->pose.pose.orientation;
    } else {
      goal.pose.orientation.w = 1.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "Found goal at: x=%.2f, y=%.2f",
      goal.pose.position.x, goal.pose.position.y);
    
    return goal;
  }

  double calculate_distance(
    const geometry_msgs::msg::Pose & pose1,
    const geometry_msgs::msg::Pose & pose2)
    
  {
    return std::sqrt(
      std::pow(pose1.position.x - pose2.position.x, 2) +
      std::pow(pose1.position.y - pose2.position.y, 2));
  }

  bool is_goal_reached()
  {
    if (!odom_) {
      return false;
    }

    double distance = calculate_distance(odom_->pose.pose, current_goal_.pose);
    return distance < goal_threshold_;
  }

  void publish_goal()
  {
    if (goal_reached_) {
      timer_->cancel();
      return;
    }

    // Update the timestamp
    current_goal_.header.stamp = this->now();
    
    // Publish the goal
    goal_publisher_->publish(current_goal_);
    
    // Check if goal is reached
    if (is_goal_reached()) {
      RCLCPP_INFO(this->get_logger(), "Goal has been reached!");
      goal_reached_ = true;
    }
  }

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FindIntersection::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), 
      "Received goal request");
    (void)uuid;
    
    // Here you could add some validation if needed
    if (!map1_ || !map2_ || !odom_) {
      RCLCPP_WARN(this->get_logger(), "Rejecting goal: missing map or odometry data");
      return rclcpp_action::GoalResponse::REJECT;
    }
    
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFindIntersection> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    
    // Stop the timer if it's running
    if (timer_) {
      timer_->cancel();
    }
    
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleFindIntersection> goal_handle)
  {
    // Create a new thread to execute the action
    std::thread{std::bind(&FindIntersectionActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleFindIntersection> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    
    // Reset goal status
    goal_reached_ = false;
    
    // Find a goal based on requested x and y
    current_goal_ = find_goal();
    if (current_goal_.header.frame_id.empty()) {
      auto result = std::make_shared<FindIntersection::Result>();
      result->success = false;
      RCLCPP_ERROR(this->get_logger(), "Failed to find intersection: missing map or odometry data");
      goal_handle->abort(result);
      return;
    }
    
    // Create a timer to publish the goal at 5 Hz until reached
    timer_ = this->create_wall_timer(
      200ms,  // 5 Hz
      std::bind(&FindIntersectionActionServer::publish_goal, this));
    
    // Prepare the feedback and result
    // auto feedback = std::make_shared<FindIntersection::Feedback>();
    auto result = std::make_shared<FindIntersection::Result>();
    
    // Keep checking goal status
    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && !goal_reached_) {
      // Check if there is a cancellation request
      if (goal_handle->is_canceling()) {
        result->success = false;
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      
      // Publish feedback
      if (odom_) {
        // feedback->distance_remaining = calculate_distance(odom_->pose.pose, current_goal_.pose);
        // goal_handle->publish_feedback(feedback);
        RCLCPP_DEBUG(this->get_logger(), "Distance to goal: %.2f meters", calculate_distance(odom_->pose.pose, current_goal_.pose));
      }
      
      loop_rate.sleep();
    }
    
    // Goal reached successfully
    if (goal_reached_) {
      result->success = true;
      RCLCPP_ERROR(this->get_logger(), "Successfully reached the intersection point");
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    } else {
      result->success = false;
      RCLCPP_ERROR(this->get_logger(), "Failed to reach the intersection point");
      goal_handle->abort(result);
      RCLCPP_INFO(this->get_logger(), "Goal aborted");
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FindIntersectionActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}