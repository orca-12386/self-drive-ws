#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose.hpp"

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
  double resolution_ = 0.08;
  int width_ = 3000;
  int height_ = 3000;
  float prior_ = 0.5;
  float prob_hit_ = 0.8;
  float prob_miss_ = 0.2;
  float min_prob_ = 0.12;
  float max_prob_ = 0.99;
  float obstacle_thresh_ = 0.25;


class PointCloudRelay : public rclcpp::Node
{
public:
    PointCloudRelay()
    : Node("pointcloud_relay")
    {

        // Publisher
        obstacle_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        free_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        
        this->declare_parameter("mask_sub_topic", "/mask/white");
        this->declare_parameter("map_pub_topic", "/map/white");
        this->declare_parameter("pc_topic","/nav/point_cloud");
        std::string map_pub_topic = this->get_parameter("map_pub_topic").as_string();
        std::string mask_sub_topic = this->get_parameter("mask_sub_topic").as_string();
        std::string pc_sub_topic = this->get_parameter("pc_topic").as_string();
        
        pub_obstacles_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/output/obstacle_cloud", 10);

        // Point cloud subscriber
        sub_pc_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pc_sub_topic,
            10,
            std::bind(&PointCloudRelay::pointcloud_callback, this, std::placeholders::_1)
        );

        // Odometry subscriber
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&PointCloudRelay::odom_callback, this, std::placeholders::_1)
        );

        // Mask subscriber
        sub_mask_ = this->create_subscription<sensor_msgs::msg::Image>(
            mask_sub_topic,
            10,
            std::bind(&PointCloudRelay::mask_callback, this, std::placeholders::_1)
        );

        occupancy_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, 10);
        
        grid_ = {resolution_, -(width_ / 2.0) * resolution_,
            -(height_ / 2.0) * resolution_, width_, height_};
        RCLCPP_WARN(this->get_logger(),"GRID INITIALIZED WITH width %d  and height: %d",grid_.width,grid_.height);

        // Initialize occupancy grid parameters
        occupancy_grid_.header.frame_id = "robot/odom"; // or "odom" when using sim 
        occupancy_grid_.info.resolution = grid_.resolution;  // meters per cell
        occupancy_grid_.info.width = grid_.width;       // 10m wide
        occupancy_grid_.info.height = grid_.height;      // 10m tall

        occupancy_grid_.info.origin.position.x = grid_.origin_x;  // Center the grid around origin
        occupancy_grid_.info.origin.position.y = grid_.origin_y;
        occupancy_grid_.info.origin.position.z = 0.0;
        occupancy_grid_.info.origin.orientation.w = 1.0;

        occupancy_grid_.data.resize(occupancy_grid_.info.width * occupancy_grid_.info.height, -1);  // Unknown

        // Create publisher


        logOddsMap_.resize(grid_.width*grid_.height,probToLogOdds(prior_));


        RCLCPP_INFO(this->get_logger(), "PointCloud relay node with odom and mask subscription started.");
    }

private:

    cv::Mat latest_mask_;
    bool mask_received_ = false;

    float probToLogOdds(float prob) { return log(prob / (1.0 - prob)); }

    float logOddsToProb(float logOdds) {
      return 1.0 - (1.0 / (1.0 + exp(logOdds)));
    }


    bool isPointOutOfBounds(const GlobalPoint &point) {
        GridCell cell = globalPointToPixelIndex(point, grid_);
        // RCLCPP_INFO(this->get_logger(), "Cell X %d Cell Y %d GP.X %f GP.y %f W %d H %d", 
            // cell.x, cell.y, point.x, point.y, grid_.width, grid_.height);
        return (cell.x < 0 || cell.x >= grid_.width || cell.y < 0 ||
                cell.y >= grid_.height);
    }


    GridCell globalPointToPixelIndex(const GlobalPoint &global_point,
                                     const OccupancyGrid &grid) {
      int x = static_cast<int>(
          round((global_point.x - grid.origin_x) / grid.resolution));
      int y = static_cast<int>(
          round((global_point.y - grid.origin_y) / grid.resolution));
      return {x, y};
    }


    void updateMap(nav_msgs::msg::OccupancyGrid &map) {
    
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
          // RCLCPP_INFO(this->get_logger(),"Checking Obstacke Cloud: %d %d ",cell.x,cell.y);
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

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
    
        if (!mask_received_)
        {
            RCLCPP_WARN(this->get_logger(), "No mask received yet. Cannot classify points.");
            return;
        }
    
        if (msg->height == 1)
        {
            RCLCPP_WARN(this->get_logger(), "Unorganized point cloud. Cannot map 2D mask.");
            return;
        }
        
        // RCLCPP_INFO(this->get_logger(),"Point Cloud Recieved pc height:%d  pc width: %d",msg->height,msg->width);
        // Resize mask to match point cloud dimensions
        cv::Mat resized_mask;
        cv::resize(latest_mask_, resized_mask, cv::Size(msg->width, msg->height), 0, 0, cv::INTER_NEAREST);
    
        // Convert PointCloud2 msg to pcl::PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
    
        // Containers for obstacles and free space
        obstacle_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        free_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    
        int width = msg->width;
        int height = msg->height;
    
        for (size_t i = 0; i < input_cloud->points.size(); ++i)
        {
            int row = i / width;
            int col = i % width;
    
            const auto& pt = input_cloud->points[i];
            if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
            
            GlobalPoint gp{
                pt.x,
                pt.y
            };
            
            if(isPointOutOfBounds(gp))
            {
                // RCLCPP_INFO(this->get_logger(),"Point Out of Bounds global point.x %d  global point.y %d   width %d   height %d",pt.x,pt.y,grid_.width,grid_.height);
                continue;
            }
            if (resized_mask.at<uchar>(row, col) == 255)
            {
                obstacle_cloud_->points.push_back(pt);
            }
            else
            {
                free_cloud_->points.push_back(pt);
            }
        }
        // RCLCPP_INFO(this->get_logger(),"MAP ABOUT TO BE PUBLISHED");
        obstacle_cloud_->width = obstacle_cloud_->points.size();
        obstacle_cloud_->height = 1;
        obstacle_cloud_->is_dense = false;
    
        free_cloud_->width = free_cloud_->points.size();
        free_cloud_->height = 1;
        free_cloud_->is_dense = false;
    
        // Convert obstacle cloud back to ROS2 msg and publish
        sensor_msgs::msg::PointCloud2 obstacle_msg;
        pcl::toROSMsg(*obstacle_cloud_, obstacle_msg);
        obstacle_msg.header = msg->header;
        pub_obstacles_->publish(obstacle_msg);
        
        updateMap(occupancy_grid_);
        occupancy_grid_.header.stamp = this->get_clock()->now();
        occupancy_pub_->publish(occupancy_grid_);
        RCLCPP_DEBUG(this->get_logger(), "Published occupancy grid.");

        // RCLCPP_INFO(this->get_logger(), "Published %zu obstacle points, %zu free points",
        //             obstacle_cloud_->points.size(), free_cloud_->points.size());
    }
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Odometry received: position = (%.2f, %.2f)",
                     msg->pose.pose.position.x, msg->pose.pose.position.y);
    }

    void mask_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
            latest_mask_ = cv_ptr->image;
            mask_received_ = true;

        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pc_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_mask_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_obstacles_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr free_cloud_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_pub_;
    rclcpp::TimerBase::SharedPtr occupancy_timer_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    BotPosition current_bot_pose_;
    std::vector<float> logOddsMap_;
    OccupancyGrid grid_;

    
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRelay>());
    rclcpp::shutdown();
    return 0;
}
