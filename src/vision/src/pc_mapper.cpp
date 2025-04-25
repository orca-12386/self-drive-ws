#include <chrono>
#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <thread>
#include <unordered_map>
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals;

class PCMapper : public rclcpp::Node {
private:
  struct GridCell {
    int x, y;
  };

  struct GlobalPoint {
    double x, y;
  };

  struct BotPosition {
    GlobalPoint global_pose;
    GridCell map_pose;
    double yaw, roll, pitch;
  };

  struct RGB {
    uint8_t r, g, b;
  };

  struct HSV {
    float h, s, v;
  };

  struct OccupancyGrid {
    double resolution;
    double origin_x, origin_y;
    int width, height;
  };

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr modify_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_sleep_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_cloud_pub_;
  
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr modify_sub_;
  
  rclcpp::TimerBase::SharedPtr timer_;


  nav_msgs::msg::OccupancyGrid map;

  float prior_;
  float prob_hit_;
  float prob_miss_;
  float min_prob_;
  float max_prob_;
  float obstacle_thresh_;
  float octree_resolution_;
  double resolution_;
  int width_;
  int height_;

  BotPosition current_bot_pose_;
  OccupancyGrid grid_;
  std::vector<float> logOddsMap_;
  cv::Mat current_mask_;
  bool mask_received_;
  std::mutex mask_mutex_;

  bool map_resizing = false;

  pcl::octree::OctreePointCloud<pcl::PointXYZ> obstacle_octree_;
  pcl::octree::OctreePointCloud<pcl::PointXYZ> free_octree_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  float probToLogOdds(float prob) { return log(prob / (1.0 - prob)); }

  float logOddsToProb(float logOdds) {
    return 1.0 - (1.0 / (1.0 + exp(logOdds)));
  }

  GridCell globalPointToPixelIndex(const GlobalPoint &global_point,
                                   const OccupancyGrid &grid) {
    int x = static_cast<int>(
        round((global_point.x - grid.origin_x) / grid.resolution));
    int y = static_cast<int>(
        round((global_point.y - grid.origin_y) / grid.resolution));
    return {x, y};
  }

  GlobalPoint cloudPointToGlobal(const pcl::PointXYZRGB &cloud_point,
                                 const BotPosition &bot_pose) {
    double cos_yaw = cos(bot_pose.yaw);
    double sin_yaw = sin(bot_pose.yaw);

    double rotated_x = cloud_point.x * cos_yaw - cloud_point.y * sin_yaw;
    double rotated_y = cloud_point.x * sin_yaw + cloud_point.y * cos_yaw;

    return {rotated_x + bot_pose.global_pose.x,
            rotated_y + bot_pose.global_pose.y};
  }

  void maskCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mask_mutex_);
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      current_mask_ = cv_ptr->image;
      mask_received_ = true;
      // RCLCPP_INFO(this->get_logger(),"Lane Mask Recieved");
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }

  void resizeMap() {
    RCLCPP_INFO(this->get_logger(), "Creating new map centered at robot position");

    grid_.origin_x =
        current_bot_pose_.global_pose.x - (width_ * resolution_ / 2.0);
    grid_.origin_y =
        current_bot_pose_.global_pose.y - (height_ * resolution_ / 2.0);

    RCLCPP_INFO(this->get_logger(), "Map initialized with origin as x %f and y %f", 
                grid_.origin_x, grid_.origin_y);

    logOddsMap_.clear();
    logOddsMap_.resize(grid_.width * grid_.height, probToLogOdds(prior_));

    nav_msgs::msg::OccupancyGrid new_map;
    new_map.info.resolution = grid_.resolution;
    new_map.info.width = grid_.width;
    new_map.info.height = grid_.height;
    new_map.info.origin.position.x = grid_.origin_x;
    new_map.info.origin.position.y = grid_.origin_y;
    new_map.info.origin.position.z = 0.0;
    new_map.info.origin.orientation.w = 1.0;

    new_map.data.clear();
    new_map.data.resize(grid_.width * grid_.height, -1);

    new_map.header.stamp = this->now();
    new_map.header.frame_id = "robot/odom";

    obstacle_cloud_->clear();
    free_cloud_->clear();
    obstacle_octree_.deleteTree();
    free_octree_.deleteTree();

    updateMap(new_map);
    map_pub_->publish(std::move(new_map));
    RCLCPP_INFO(this->get_logger(), "New map created at robot position (%.2f, %.2f)",
                current_bot_pose_.global_pose.x, current_bot_pose_.global_pose.y);
  }

  bool isPointOutOfBounds(const GlobalPoint &point) {
    GridCell cell = globalPointToPixelIndex(point, grid_);
    // RCLCPP_WARN(this->get_logger(),"Cell X %d Cell Y: %d Width %d Height: %d",cell.x,cell.y,grid_.width,grid_.height);
    return (cell.x < 0 || cell.x >= grid_.width || cell.y < 0 ||
            cell.y >= grid_.height);
  }

  void resizeMapCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "resize") {
      map_resizing = true;

      auto sleep_msg = std::make_unique<std_msgs::msg::String>();
      sleep_msg->data = "sleep";
      goal_sleep_pub_->publish(std::move(sleep_msg));

      resizeMap();
    }
  }

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    if (map_resizing) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                           "Map resizing, skipping cloud data");
      return;
    }

    if (!mask_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "No mask received yet. Skipping point cloud processing.");
      return;
    }

    std::lock_guard<std::mutex> lock(mask_mutex_);

    obstacle_cloud_->clear();
    free_cloud_->clear();
    obstacle_octree_.deleteTree();
    free_octree_.deleteTree();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *current_cloud);

    int cloud_width = msg->width;
    int cloud_height = msg->height;
    
    if (cloud_height != current_mask_.rows ||
        cloud_width != current_mask_.cols) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Mask and point cloud dimensions don't match! Mask: "
                           "%dx%d, Cloud: %dx%d",
                           current_mask_.rows, current_mask_.cols, cloud_height,
                           cloud_width);
    
      cv::resize(current_mask_,current_mask_, cv::Size(cloud_height, cloud_width));  // Resize to 640x480
                           
      // return;
    }

    bool needs_resize = false;
    int i = 0, obstacle_count = 0;
    RCLCPP_WARN(this->get_logger(),"Point Cloud Recieved:");
    for (int row = 0; row < cloud_height; ++row) {
      for (int col = 0; col < cloud_width; ++col) {
        i++;
        int index = row * cloud_width + col;
        const auto &point = current_cloud->points[index];

        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          continue;
        }

        // GlobalPoint global_point = cloudPointToGlobal(point, current_bot_pose_);
        GlobalPoint global_point;
        global_point.x = point.x;
        global_point.y = point.y;

        pcl::PointXYZ pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = 0.0;

        uchar mask_value = current_mask_.at<uchar>(row, col);

        if (point.z < 0.1) {

          if (isPointOutOfBounds(global_point)) {
            needs_resize = true;
            break;
          }

          if (mask_value > 127) {
            obstacle_count += 1;
            obstacle_cloud_->push_back(pcl_point);
          } else {
            if (i % 10 == 0) {
              free_cloud_->push_back(pcl_point);
            }
          }
        }
      }
      if (needs_resize)
        break;
    }

    if (needs_resize) {
      auto resize_msg = std::make_unique<std_msgs::msg::String>();
      resize_msg->data = "resize";
      modify_pub_->publish(std::move(resize_msg));
      return;
    }

    obstacle_octree_.setInputCloud(obstacle_cloud_);
    obstacle_octree_.addPointsFromInputCloud();

    free_octree_.setInputCloud(free_cloud_);
    free_octree_.addPointsFromInputCloud();

    auto debug_cloud = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*obstacle_cloud_, *debug_cloud);
    debug_cloud->header.frame_id = "robot/odom";
    debug_cloud->header.stamp = this->now();
    debug_cloud_pub_->publish(std::move(debug_cloud));
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_bot_pose_.global_pose.x = msg->pose.pose.position.x;
    current_bot_pose_.global_pose.y = msg->pose.pose.position.y;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3(q).getRPY(current_bot_pose_.roll, current_bot_pose_.pitch,
                             current_bot_pose_.yaw);
  }

  void updateMap(nav_msgs::msg::OccupancyGrid map) {
    if (map_resizing)
      return;

    std::unordered_map<int, int> free_count;
    std::unordered_map<int, int> obstacle_count;
    std::unordered_map<int, float> free_accumulated;
    std::unordered_map<int, float> obstacle_accumulated;

    for (const auto &point : free_cloud_->points) {
      GlobalPoint global_point{point.x, point.y};
      GridCell cell = globalPointToPixelIndex(global_point, grid_);

      if (cell.x >= 0 && cell.x < grid_.width && cell.y >= 0 &&
          cell.y < grid_.height) {
        int index = cell.y * grid_.width + cell.x;
        free_count[index]++;
        free_accumulated[index] += probToLogOdds(prob_miss_);
      }
    }

    for (const auto &point : obstacle_cloud_->points) {
      GlobalPoint global_point{point.x, point.y};
      GridCell cell = globalPointToPixelIndex(global_point, grid_);

      if (cell.x >= 0 && cell.x < grid_.width && cell.y >= 0 &&
          cell.y < grid_.height) {
        int index = cell.y * grid_.width + cell.x;
        obstacle_count[index]++;
        obstacle_accumulated[index] += probToLogOdds(prob_hit_);
      }
    }

    for (const auto &[index, count] : free_count) {
      float average_update = free_accumulated[index] / count;
      logOddsMap_[index] += average_update;

      float prob = logOddsToProb(logOddsMap_[index]);
      prob = std::max(min_prob_, std::min(max_prob_, prob));
      logOddsMap_[index] = probToLogOdds(prob);
      map.data[index] = (prob >= obstacle_thresh_) ? 100 : 0;
    }

    for (const auto &[index, count] : obstacle_count) {
      float average_update = obstacle_accumulated[index] / count;
      logOddsMap_[index] += average_update;

      float prob = logOddsToProb(logOddsMap_[index]);
      prob = std::max(min_prob_, std::min(max_prob_, prob));
      logOddsMap_[index] = probToLogOdds(prob);
      map.data[index] = (prob >= obstacle_thresh_) ? 100 : 0;
    }
  }


  void loadParameters() {
    // Declare and get parameters using ROS2 parameter system
    this->declare_parameter("prior", 0.5);
    this->declare_parameter("prob_hit", 0.6);
    this->declare_parameter("prob_miss", 0.3);
    this->declare_parameter("min_prob", 0.12);
    this->declare_parameter("max_prob", 0.97);
    this->declare_parameter("obstacle_threshold", 0.6);
    this->declare_parameter("octree_resolution", 0.01);
    this->declare_parameter("resolution", 0.01);
    this->declare_parameter("width", 3500);
    this->declare_parameter("height", 3500);

    prior_ = this->get_parameter("prior").as_double();
    prob_hit_ = this->get_parameter("prob_hit").as_double();
    prob_miss_ = this->get_parameter("prob_miss").as_double();
    min_prob_ = this->get_parameter("min_prob").as_double();
    max_prob_ = this->get_parameter("max_prob").as_double();
    obstacle_thresh_ = this->get_parameter("obstacle_threshold").as_double();
    octree_resolution_ = this->get_parameter("octree_resolution").as_double();
    resolution_ = this->get_parameter("resolution").as_double();
    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
  }

  void timerCallback() {

        map.header.stamp = this->now();
        map.header.frame_id = "/robot/odom";

        map.info.width = grid_.width;
        map.info.height = grid_.height;
        map.info.origin.position.x = grid_.origin_x;
        map.info.origin.position.y = grid_.origin_y;

        if (map_resizing) {
          RCLCPP_INFO(this->get_logger(),"Resizing Map so Clearing Data");
          obstacle_cloud_->clear();
          free_cloud_->clear();
          obstacle_octree_.deleteTree();
          free_octree_.deleteTree();
          map.data.clear();
          map_resizing = false;

          map.data.resize(grid_.width * grid_.height, -1);

          return;
        }

        updateMap(map);
        map_pub_->publish(map);
    
  }

public:
  PCMapper() 
      : Node("pc_mapper_node"),
        obstacle_octree_(0.01), free_octree_(0.01),
        obstacle_cloud_(new pcl::PointCloud<pcl::PointXYZ>),
        free_cloud_(new pcl::PointCloud<pcl::PointXYZ>), 
        mask_received_(false) {

    loadParameters();

    // Initialize TF2
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create subscribers
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "pointcloud_topic_sub", 
      rclcpp::QoS(1),
      std::bind(&PCMapper::pointCloudCallback, this, std::placeholders::_1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom_topic_sub", 
      rclcpp::QoS(1),
      std::bind(&PCMapper::odomCallback, this, std::placeholders::_1));
    
    mask_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "mask_topic_sub", 
      rclcpp::QoS(1),
      std::bind(&PCMapper::maskCallback, this, std::placeholders::_1));
    
    modify_sub_ = this->create_subscription<std_msgs::msg::String>(
      "map_modify_sub", 
      rclcpp::QoS(1),
      std::bind(&PCMapper::resizeMapCallback, this, std::placeholders::_1));

    // Create publishers
    modify_pub_ = this->create_publisher<std_msgs::msg::String>("map_modify_pub", rclcpp::QoS(1));
    goal_sleep_pub_ = this->create_publisher<std_msgs::msg::String>("goal_sleep_pub", rclcpp::QoS(1).transient_local());
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_pub", rclcpp::QoS(1).transient_local());
    debug_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("debug_cloud_pub", rclcpp::QoS(1).transient_local());

    // Initialize grid
    grid_ = {resolution_, -(width_ / 2.0) * resolution_,
             -(height_ / 2.0) * resolution_, width_, height_};


    map.info.resolution = grid_.resolution;
    map.info.width = grid_.width;
    map.info.height = grid_.height;
    map.info.origin.position.x = grid_.origin_x;
    map.info.origin.position.y = grid_.origin_y;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    logOddsMap_.resize(grid_.width * grid_.height, probToLogOdds(prior_));

    // Create timer for main loop
    timer_ = this->create_wall_timer(50ms, std::bind(&PCMapper::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "PointCloud Mapper Node Started In sim.");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PCMapper>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}