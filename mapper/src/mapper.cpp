#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree_search.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <vector>
#include <cmath>

class Mapper : public rclcpp::Node
{
public:
    Mapper() 
        : Node("Mapper"),
          occupied_cloud(new pcl::PointCloud<pcl::PointXYZ>),
          unoccupied_cloud(new pcl::PointCloud<pcl::PointXYZ>),
          occupied_octree(resolution),
          unoccupied_octree(resolution)
    {
        RCLCPP_INFO(this->get_logger(), "Mapper Node Started.");

        // Initialize subscriptions
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed_node/stereocamera/points", 1, 
            std::bind(&Mapper::pointCloudCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 1, std::bind(&Mapper::odomCallback, this, std::placeholders::_1));

        // Initialize publisher
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", rclcpp::SystemDefaultsQoS());

        // Initialize timer
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10), std::bind(&Mapper::UpdateFunction, this));

        // Initialize grid and maps
        grid.width = 1500;
        grid.height = 1000;
        grid.resolution = resolution;
        grid.origin_x = -(grid.width/2)*grid.resolution;
        grid.origin_y = -(grid.height/2)*grid.resolution;

        log_odds_map.resize(grid.width * grid.height, L_PRIOR);
        hit_count.resize(grid.width * grid.height, 0);
        update_sum.resize(grid.width * grid.height, 0.0);
    }

private:
    struct BotPosition {
        double x, y, z;
        double yaw, roll, pitch;
    };

    struct GridCell {
        int x, y;
    };

    struct OccupancyGrid {
        double resolution;
        double origin_x, origin_y;
        int width, height;
    };

    const double resolution = 0.06;
    const double P_PRIOR = 0.5;  
    const double P_OCC = 0.7;    
    const double P_FREE = 0.3;   
    const double L_OCC = log(P_OCC / (1 - P_OCC));
    const double L_FREE = log(P_FREE / (1 - P_FREE));
    const double L_PRIOR = log(P_PRIOR / (1 - P_PRIOR));
    const double L_MIN = -5;  
    const double L_MAX = 5;   

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr occupied_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr unoccupied_cloud;

    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> occupied_octree;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> unoccupied_octree;

    BotPosition current_bot_pose;
    OccupancyGrid grid;

    bool cloud_received = false;
    bool odom_received = false;

    std::vector<double> log_odds_map;
    std::vector<int> hit_count;
    std::vector<double> update_sum;

    pcl::PointXYZ cloudPointToGlobalPoint(const pcl::PointXYZ& cloud_point, const BotPosition& bot_pos) {
        pcl::PointXYZ global_point;

        global_point.x = bot_pos.x + cloud_point.x*sin(bot_pos.yaw) + cloud_point.z*cos(bot_pos.yaw);
        global_point.y = bot_pos.y + cloud_point.z*sin(bot_pos.yaw) - cloud_point.x*cos(bot_pos.yaw);
        global_point.z = cloud_point.y;

        return global_point;
    }

    GridCell globalPointToPixelIndex(const pcl::PointXYZ& global_point, const OccupancyGrid& grid) {
        int x = static_cast<int>(round((global_point.x - grid.origin_x)/grid.resolution));
        int y = static_cast<int>(round((global_point.y - grid.origin_y)/grid.resolution));

        return {x, y};
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromROSMsg(*msg, *current_cloud);

        occupied_cloud->clear();
        unoccupied_cloud->clear();
        
        for (int i = current_cloud->size() - 1; i >= 0; i--) {
            pcl::PointXYZ cloud_point;

            uint8_t r = current_cloud->points[i].r;
            uint8_t g = current_cloud->points[i].g;
            uint8_t b = current_cloud->points[i].b;

            float x = current_cloud->points[i].x;
            float y = current_cloud->points[i].y;
            float z = current_cloud->points[i].z;

            cloud_point.x = x;
            cloud_point.y = y;
            cloud_point.z = z;

            if (x < 6  && x > -6 && z < 8) {
                if ((r > 170 && g > 170 && b > 170 && z < 8 && r<190 && g<190 && b<190)) {
                    if (i%1 == 0)
                        occupied_cloud->points.push_back(cloud_point);
                } else {
                    if (i%2 == 0)
                        unoccupied_cloud->points.push_back(cloud_point);
                }
            }
        }

        occupied_octree.deleteTree();
        occupied_octree.setInputCloud(occupied_cloud);
        occupied_octree.addPointsFromInputCloud();

        unoccupied_octree.deleteTree();
        unoccupied_octree.setInputCloud(unoccupied_cloud);
        unoccupied_octree.addPointsFromInputCloud();

        cloud_received = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_bot_pose.x = msg->pose.pose.position.x;
        current_bot_pose.y = msg->pose.pose.position.y;
        current_bot_pose.z = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3(q).getRPY(current_bot_pose.roll, current_bot_pose.pitch, current_bot_pose.yaw);

        odom_received = true;
    }

    void UpdateFunction()
    {
        if (!odom_received || !cloud_received) {
            return;
        }

        updateOccupancyGrid();
        publishOccupancyGrid();

        cloud_received = false;
        odom_received = false;
    }

    void updateOccupancyGrid() {
        std::fill(hit_count.begin(), hit_count.end(), 0);
        std::fill(update_sum.begin(), update_sum.end(), 0.0);

        updateGridWithCloud(unoccupied_octree, L_FREE);
        updateGridWithCloud(occupied_octree, L_OCC);

        for (size_t i = 0; i < log_odds_map.size(); ++i) {
            if (hit_count[i] > 0) {
                double avg_update = update_sum[i] / hit_count[i];
                log_odds_map[i] = std::max(L_MIN, std::min(L_MAX, log_odds_map[i] + avg_update));
            }
        }
    }

    void updateGridWithCloud(const pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>& octree, double update_value) {
        std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> centers;
        octree.getOccupiedVoxelCenters(centers);

        for (const auto& point : centers) {
            pcl::PointXYZ global_point = cloudPointToGlobalPoint(point, current_bot_pose);
            GridCell indices = globalPointToPixelIndex(global_point, grid);

            if (indices.x >= 0 && indices.x < grid.width && indices.y >= 0 && indices.y < grid.height) {
                int index = indices.y * grid.width + indices.x;
                update_sum[index] += update_value;
                hit_count[index]++;
            }
        }
    }

    void publishOccupancyGrid() {
        auto map = std::make_unique<nav_msgs::msg::OccupancyGrid>();
        map->info.resolution = grid.resolution;
        map->info.width = grid.width;
        map->info.height = grid.height;
        map->info.origin.position.x = grid.origin_x;
        map->info.origin.position.y = grid.origin_y;
        map->info.origin.position.z = 0.0;
        map->info.origin.orientation.w = 1.0;
        map->data.resize(grid.width * grid.height, -1);

        for (size_t i = 0; i < map->data.size(); ++i) {
            double prob = 1 - (1 / (1 + exp(log_odds_map[i])));
            if (prob > 0.7) {
                map->data[i] = 100;
            } else if (prob < 0.3) {
                map->data[i] = 0;
            } else {
                map->data[i] = -1;
            }
        }

        map->header.stamp = this->now();
        map->header.frame_id = "map";
        map_pub_->publish(std::move(map));
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mapper>());
    rclcpp::shutdown();
    return 0;
}
