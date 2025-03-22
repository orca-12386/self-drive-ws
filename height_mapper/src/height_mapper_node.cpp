#define HEIGHT 3000
#define WIDTH 3000
#define RESOLUTION 0.08

// #define DEBUG

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <cmath>


class Timer {
public:
    Timer(const std::string& s) : name(s) {
        begin = std::chrono::steady_clock::now();
    }

    int elapsed() {
        end = std::chrono::steady_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
        return static_cast<int>(duration);
    }

    std::string log() {
        int duration = this->elapsed();
        return name+std::string(": ")+std::to_string(duration);
    }

private:
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point end;
    std::string name;
};


class HeightMapperNode : public rclcpp::Node
{
public:
    HeightMapperNode() : rclcpp::Node("height_mapper_node") {
        RCLCPP_INFO(this->get_logger(), "height_mapper_node started");
        // Initialise subscriptions
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/depth/image_raw", 10, std::bind(&HeightMapperNode::depthImageCallback, this, std::placeholders::_1));

        mask_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/height_mask", 10, std::bind(&HeightMapperNode::maskImageCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed_node/stereocamera/camera_info", 10, std::bind(&HeightMapperNode::cameraInfoCallback, this, std::placeholders::_1));

        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&HeightMapperNode::odomCallback, this, std::placeholders::_1));


        // Initialise subscription flags
        depth_recv = false;
        mask_recv = false;
        camera_info_recv = false;
        odometry_recv = false;

        // Initialise publishers
        rclcpp::QoS qos_settings = rclcpp::QoS(rclcpp::KeepLast(20)).transient_local();
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/height_map", qos_settings);

        // Initialise map
        grid_resolution = RESOLUTION;
        grid_height = HEIGHT;
        grid_width = WIDTH;
        grid_origin_x = -(WIDTH/2.0)*RESOLUTION;
        grid_origin_y = -(HEIGHT/2.0)*RESOLUTION;

        z_threshold = 20;
        
        prob_mark = 0.7;
        log_odds_mark = prob_to_log_odds(prob_mark);
        prob_hit = 0.7;
        log_odds_hit = prob_to_log_odds(prob_hit);
        prob_miss = 0.3;
        log_odds_miss = prob_to_log_odds(prob_miss);
        prob_unknown = 0.5;
        log_odds_unknown = prob_to_log_odds(prob_unknown);

        log_odds_upper = 2.0;
        log_odds_lower = -1.0;

        for(int i = 0;i<grid_height ;i++) {
            for(int j = 0;j<grid_width;j++) {
                log_odds_map[(i*grid_width) + j] = log_odds_unknown;
            }
        }

        height_map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        height_map_msg->data.resize(grid_height*grid_width, 0);
        height_map_msg->header.frame_id = "map";
        height_map_msg->info.width = grid_width;
        height_map_msg->info.height = grid_height;
        height_map_msg->info.resolution = grid_resolution;
        height_map_msg->info.origin.position.x = grid_origin_x;
        height_map_msg->info.origin.position.y = grid_origin_y;
        height_map_msg->info.origin.position.z = 0.0;
        height_map_msg->info.origin.orientation.w = 1.0;

        timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&HeightMapperNode::timer_callback, this));
        
    };

private:
    // Callback functions

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->depth_image_msg = msg;
        depth_recv = true;
    }

    void maskImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->mask_image_msg = msg;
        mask_recv = true;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->camera_info_msg = msg;
        camera_info_recv = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        odometry_recv = true;
    }

    void log_debug(std::string str) {
#ifdef DEBUG
        RCLCPP_INFO(this->get_logger(), str.c_str());
#else
        return;
#endif
    }
    
    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    // Points

    typedef struct Point {
        double x;
        double y;
        double z;
        double local_grid_x;
        double local_grid_y;
        int global_grid_x;
        int global_grid_y;
    } Point;

    Point convert_depth_to_point(const cv::Point& location, const double& depth, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        double fx = camera_info->k[0]; 
        double fy = camera_info->k[4];
        double cx = camera_info->k[2];
        double cy = camera_info->k[5];
        double u = location.x;
        double v = location.y;
        double z = depth;
        double x = ((u-cx)*z)/fx;
        double y = ((v-cy)*z)/fy;
        Point p = Point();
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    void convert_to_grid_coords(Point& p, double refx, double refy) {
        p.local_grid_x = (p.x - refx)/grid_resolution;
        p.local_grid_y = (p.z - refy)/grid_resolution; 
    }

    void rotate_local_grid(Point& p, double yaw) {
        double original_y = p.local_grid_y;
        p.local_grid_y = (p.local_grid_y*sin(yaw)) - (p.local_grid_x*cos(yaw)); 
        p.local_grid_x = (p.local_grid_x*sin(yaw)) + (original_y*cos(yaw));
    }

    void get_global_grid_coords(Point& p, double shifted_origin_x, double shifted_origin_y) {
        p.global_grid_x = static_cast<int>(std::round(p.local_grid_x + shifted_origin_x));
        p.global_grid_y = static_cast<int>(std::round(p.local_grid_y + shifted_origin_y));
    }

    double log_odds_to_prob(const double& log_odds) {
        return 1-(1/(1+exp(log_odds)));
    }

    double prob_to_log_odds(const double& prob) {
        return std::log(prob/(1-prob));
    }

    void get_points(sensor_msgs::msg::Image::SharedPtr mask, sensor_msgs::msg::Image::SharedPtr depth, sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {    

        Timer t = Timer("sensor msg to cv mat");

        depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        depth_image = depth_image_ptr->image;

        mask_image_ptr = cv_bridge::toCvCopy(mask, mask->encoding);
        mask_image = mask_image_ptr->image;

        if(mask_image.empty()) {
            return;
        } 
        if(depth_image.empty()) {
            return;
        }

        log_debug(t.log());

        Timer t3 = Timer("Locations");
        // Mask depth image and convert to point format
        locations.clear();
        cv::findNonZero(mask_image, locations); // Get all nonzero pixel locations
        log_debug(std::string("Number of non-zero pixel locations: ")+std::to_string(static_cast<int>(locations.size())));
       
        log_debug(t3.log());

        Timer t4 = Timer("Point vector");

        points.clear();
        for (const auto& pt : locations) {
            double depth = static_cast<double>(depth_image.at<float>(pt.y, pt.x)); // Access color pixel
            Point p = convert_depth_to_point(pt, depth, camera_info);
            if(p.z<z_threshold) {
                points.push_back(p);
            }
        }
        log_debug(std::string("Number of points: ")+std::to_string(static_cast<int>(points.size())));

        log_debug(t4.log());

        Timer t5 = Timer("Odometry");

        Point odom;
        odom.x = odometry_msg->pose.pose.position.x;
        odom.y = odometry_msg->pose.pose.position.z;
        odom.z = odometry_msg->pose.pose.position.y;
        convert_to_grid_coords(odom, grid_origin_x, grid_origin_y);
        get_global_grid_coords(odom, 0, 0);
        // instant_map_msg->data[odom.global_grid_x+(grid_width*odom.global_grid_y)] = 100;

        //log_debug("Converted odom to grid coords");

        tf2::Quaternion q(
            odometry_msg->pose.pose.orientation.x,
            odometry_msg->pose.pose.orientation.y,
            odometry_msg->pose.pose.orientation.z,
            odometry_msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        //log_debug("Calculated yaw");
        log_debug(t5.log());

        std::fill(height_map_msg->data.begin(), height_map_msg->data.end(), 0);

        Timer t6 = Timer("grid conversion");
        
        int min_grid_x = grid_width, max_grid_x = 0;
        int min_grid_y = grid_height, max_grid_y = 0;
        for (auto it = std::begin (points); it != std::end (points); ++it) {
            convert_to_grid_coords(*it, 0, 0);
            rotate_local_grid(*it, yaw);
            get_global_grid_coords(*it, odom.global_grid_x, odom.global_grid_y);
            if(it->global_grid_y < grid_height && it->global_grid_y >= 0 && it->global_grid_x >= 0 && it->global_grid_x < grid_width) {
                if(it->global_grid_x < min_grid_x) {
                    min_grid_x = it->global_grid_x;
                }
                if(it->global_grid_x > max_grid_x) {
                    max_grid_x = it->global_grid_x;
                }
                if(it->global_grid_y < min_grid_y) {
                    min_grid_y = it->global_grid_y;
                }
                if(it->global_grid_y > max_grid_y) {
                    max_grid_y = it->global_grid_y;
                }
                size_t index = (it->global_grid_y*grid_width)+it->global_grid_x;
                int height = static_cast<int>(it->y*10);
                if(height > height_map_msg->data[index]) {
                    height_map_msg->data[index] = height;
                }
                log_odds_map[index] += log_odds_hit-log_odds_miss;
            } else {
                log("Assignment exceeds map dimensions");
            }
            // log_debug("d");
        }

        for(int i = min_grid_y; i<max_grid_y ;i++) {
            for(int j = min_grid_x;j<max_grid_x;j++) {
                size_t index = (i*grid_width)+j;
                log_odds_map[index] += log_odds_miss;
                if(log_odds_map[index] > log_odds_upper) {
                    log_odds_map[index] = log_odds_upper;
                }
                if(log_odds_map[index] < log_odds_lower) {
                    log_odds_map[index] = log_odds_lower;
                }
                if(log_odds_map[index] < log_odds_mark) {
                    height_map_msg->data[index] = 0;
                }
            }
        }

        height_map_msg->header.stamp = this->now();
        map_pub->publish(*height_map_msg);

    }

    void timer_callback() {
        bool recv = depth_recv;
        recv = recv && mask_recv; 
        recv = recv && camera_info_recv;
        recv = recv && odometry_recv;
        if(recv) {
            get_points(mask_image_msg, depth_image_msg, camera_info_msg);
        } else {
            //log_debug("Waiting for subscriptions");
        }
    }

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr mask_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

    // Subscription flags
    bool depth_recv;
    bool mask_recv;
    bool camera_info_recv;
    bool odometry_recv;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub;

    // Data containers
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    sensor_msgs::msg::Image::SharedPtr depth_image_msg;
    sensor_msgs::msg::Image::SharedPtr mask_image_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;

    rclcpp::TimerBase::SharedPtr timer;

    double prob_hit, prob_miss, prob_unknown, prob_mark;
    double log_odds_hit, log_odds_miss, log_odds_unknown, log_odds_mark, log_odds_upper, log_odds_lower;
    double log_odds_map[HEIGHT*WIDTH];
    
    int grid_height;
    int grid_width;
    double grid_resolution;
    double grid_origin_x;
    double grid_origin_y;

    int y_threshold, z_threshold;

    cv_bridge::CvImagePtr depth_image_ptr, mask_image_ptr;
    cv::Mat depth_image, mask_image;
    std::vector<cv::Point> locations;
    std::vector<Point> points;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> height_map_msg;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}