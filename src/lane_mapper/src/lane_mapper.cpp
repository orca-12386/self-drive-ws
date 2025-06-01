#define HEIGHT 3000
#define WIDTH 3000
#define RESOLUTION 0.04

// #define DEBUG

#include <rclcpp/rclcpp.hpp>

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


#include <vector>
#include <algorithm>
#include <cmath>
#include <nav_msgs/msg/occupancy_grid.hpp>


typedef struct Point {
    double x;
    double y;
    double z;
    double local_grid_x;
    double local_grid_y;
    int global_grid_x;
    int global_grid_y;
} Point;


bool isPointInConvexPolygon(const Point& point, const std::vector<Point>& polygon) {
    int n = polygon.size();
    
    // Need at least 3 points to form a polygon
    if (n < 3) return false;
    
    // Check if point is on the same side of all edges
    for (int i = 0; i < n; i++) {
        const Point& p1 = polygon[i];
        const Point& p2 = polygon[(i + 1) % n];
        
        // Calculate the cross product to determine side
        double crossProduct = (p2.global_grid_x - p1.global_grid_x) * (point.global_grid_y - p1.global_grid_y) - (p2.global_grid_y - p1.global_grid_y) * (point.global_grid_x - p1.global_grid_x);
        
        // If point is on the wrong side of any edge, it's outside
        if (crossProduct > 0) return false;

    }
    
    return true;
}


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


class LaneMapperNode : public rclcpp::Node
{
public:
    LaneMapperNode() : rclcpp::Node("lane_mapper_node") {
        RCLCPP_INFO(this->get_logger(), "lane_mapper_node started");

        this->declare_parameter("sim", rclcpp::PARAMETER_BOOL);
        sim = this->get_parameter("sim").as_bool();

        this->declare_parameter<std::string>("mask_sub_topic", "/mask/white");
        this->declare_parameter<std::string>("map_pub_topic", "/map/white");
        this->declare_parameter<std::string>("depth_sub_topic", "/zed/zed_node/depth/depth_registered");
        this->declare_parameter<std::string>("color_sub_topic", "/zed/zed_node/rgb/image_rect_color");
        this->declare_parameter<std::string>("camera_info_sub_topic", "/zed/zed_node/rgb/camera_info");
        
        std::string mask_sub_topic = this->get_parameter("mask_sub_topic").as_string();
        std::string map_pub_topic = this->get_parameter("map_pub_topic").as_string();
        std::string depth_sub_topic = this->get_parameter("depth_sub_topic").as_string();
        std::string color_sub_topic = this->get_parameter("color_sub_topic").as_string();
        std::string camera_info_sub_topic = this->get_parameter("camera_info_sub_topic").as_string();
        

        // Initialise subscriptions

        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            depth_sub_topic, 10, std::bind(&LaneMapperNode::depthImageCallback, this, std::placeholders::_1));

        mask_sub = this->create_subscription<sensor_msgs::msg::Image>(
            mask_sub_topic, 10, std::bind(&LaneMapperNode::maskImageCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_sub_topic, 10, std::bind(&LaneMapperNode::cameraInfoCallback, this, std::placeholders::_1));

        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&LaneMapperNode::odomCallback, this, std::placeholders::_1));


        // Initialise subscription flags
        depth_recv = false;
        mask_recv = false;
        camera_info_recv = false;
        odometry_recv = false;

        // Initialise publishers
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, 10);
        instant_map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic+std::string("/instant"), 10);

        // Initialise map
        grid_resolution = RESOLUTION;
        grid_height = HEIGHT;
        grid_width = WIDTH;
        grid_origin_x = -(WIDTH/2.0)*RESOLUTION;
        grid_origin_y = -(HEIGHT/2.0)*RESOLUTION;

        z_threshold = 10;
        y_threshold = 0.1;
        
        prob_mark = 0.55;
        log_odds_mark = prob_to_log_odds(prob_mark);
        prob_hit = 0.8;
        log_odds_hit = prob_to_log_odds(prob_hit);
        prob_miss = 0.2;
        log_odds_miss = prob_to_log_odds(prob_miss);
        prob_unknown = 0.5;
        log_odds_unknown = prob_to_log_odds(prob_unknown);

        log_odds_lower_clamp = prob_to_log_odds(0.12);
        log_odds_upper_clamp = prob_to_log_odds(0.99);

        for(int i = 0;i<grid_height ;i++) {
            for(int j = 0;j<grid_width;j++) {
                log_odds_map[(i*grid_width) + j] = log_odds_unknown;
            }
        }

        instant_map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        instant_map_msg->data.resize(grid_height*grid_width, 0);
        instant_map_msg->header.frame_id = "odom";
        instant_map_msg->info.width = grid_width;
        instant_map_msg->info.height = grid_height;
        instant_map_msg->info.resolution = grid_resolution;
        instant_map_msg->info.origin.position.x = grid_origin_x;
        instant_map_msg->info.origin.position.y = grid_origin_y;
        instant_map_msg->info.origin.position.z = 0.0;
        instant_map_msg->info.origin.orientation.w = 1.0;

        full_map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        full_map_msg->data.resize(grid_height*grid_width, -1);
        full_map_msg->header.frame_id = "odom";
        full_map_msg->info.width = grid_width;
        full_map_msg->info.height = grid_height;
        full_map_msg->info.resolution = grid_resolution;
        full_map_msg->info.origin.position.x = grid_origin_x;
        full_map_msg->info.origin.position.y = grid_origin_y;
        full_map_msg->info.origin.position.z = 0.0;
        full_map_msg->info.origin.orientation.w = 1.0;
        
    };

private:
    // Callback functions

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->depth_image_msg = msg;
        depth_recv = true;
        timer_callback();
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
        if(this->sim) {
            const float pitch = 23.5f * M_PI / 180.0f;
            const float cos_pitch = cos(pitch);
            const float sin_pitch = sin(pitch);
            y = 1.5f - z * sin_pitch - y * cos_pitch;
        }
        Point p = Point();
        p.x = x;
        p.y = y;
        p.z = z;
        //log_debug(std::string("convert_depth_to_point: ")+std::to_string(p.x)+std::string(",")+std::to_string(p.z));
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
            double d = static_cast<double>(depth_image.at<float>(pt.y, pt.x)); // Access color pixel
            Point p = convert_depth_to_point(pt, d, camera_info);
            if(p.y < y_threshold && p.z<z_threshold) {
                points.push_back(p);
            }
        }

        // get antipoints
        antipoints.clear();
        cv::Mat complement_mask_image;
        cv::bitwise_not(mask_image, complement_mask_image);
        cv::findNonZero(complement_mask_image, locations);
        for (const auto& pt : locations) {
            double d = static_cast<double>(depth_image.at<float>(pt.y, pt.x));
            Point p = convert_depth_to_point(pt, d, camera_info);
            if(p.y < y_threshold && p.z<z_threshold) {
                antipoints.push_back(p);
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

        tf2::Quaternion q(
            odometry_msg->pose.pose.orientation.x,
            odometry_msg->pose.pose.orientation.y,
            odometry_msg->pose.pose.orientation.z,
            odometry_msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        log_debug(t5.log());

        Timer t6 = Timer("grid conversion");
        
        std::fill(instant_map_msg->data.begin(), instant_map_msg->data.end(), 0);

        for (auto it = begin (points); it != end (points); ++it) {
            convert_to_grid_coords(*it, 0, 0);
            rotate_local_grid(*it, yaw);
            get_global_grid_coords(*it, odom.global_grid_x, odom.global_grid_y);
            if(it->global_grid_y < grid_height && it->global_grid_y >= 0 && it->global_grid_x >= 0 && it->global_grid_x < grid_width) {
                size_t index = (it->global_grid_y*grid_width)+it->global_grid_x;
                instant_map_msg->data[index] = 100;
                log_odds_map[index] += log_odds_hit;
                if(log_odds_map[index] > log_odds_upper_clamp) {
                    log_odds_map[index] = log_odds_upper_clamp;    
                }
                if(log_odds_map[index] < log_odds_lower_clamp) {
                    log_odds_map[index] = log_odds_lower_clamp;
                }
                if(log_odds_map[index] >= log_odds_mark) {
                    full_map_msg->data[index] = 100;
                } else if(log_odds_map[index] > log_odds_miss) {
                    full_map_msg->data[index] = -1;
                } else {
                    full_map_msg->data[index] = 0;
                }    
            } else {
                relocate();
                return;
            }
        }

        log_debug(t6.log());

        Timer t9 = Timer("polygon");

        for (auto it = begin (antipoints); it != end (antipoints); ++it) {
            convert_to_grid_coords(*it, 0, 0);
            rotate_local_grid(*it, yaw);
            get_global_grid_coords(*it, odom.global_grid_x, odom.global_grid_y);
            if(it->global_grid_y < grid_height && it->global_grid_y >= 0 && it->global_grid_x >= 0 && it->global_grid_x < grid_width) {
                size_t index = (it->global_grid_y*grid_width)+it->global_grid_x;
                instant_map_msg->data[index] = 0;
                log_odds_map[index] += log_odds_miss;
                if(abs(log_odds_map[index]) > 5) {
                    log_odds_map[index] = (abs(log_odds_map[index])/log_odds_map[index])*5;    
                }
                if(log_odds_map[index] >= log_odds_mark) {
                    full_map_msg->data[index] = 100;
                } else if(log_odds_map[index] > log_odds_miss) {
                    full_map_msg->data[index] = -1;
                } else {
                    full_map_msg->data[index] = 0;
                }    
            } else {
                relocate();
                return;
            }
        }


        log_debug(t9.log());

        Timer t7 = Timer("create and publish");

        full_map_msg->header.stamp = this->now();

        instant_map_pub->publish(*instant_map_msg);
        map_pub->publish(*full_map_msg);

        log_debug(t7.log());
    }

    void relocate() {
        // Update class member variables
        grid_origin_x = odometry_msg->pose.pose.position.x - (WIDTH * RESOLUTION / 2.0);
        grid_origin_y = odometry_msg->pose.pose.position.y - (HEIGHT * RESOLUTION / 2.0);
        
        // Clear and resize maps
        instant_map_msg->data.clear();
        instant_map_msg->data.resize(grid_height*grid_width, 0);
        instant_map_msg->info.origin.position.x = grid_origin_x;
        instant_map_msg->info.origin.position.y = grid_origin_y;
        
        full_map_msg->data.clear();
        full_map_msg->data.resize(grid_height*grid_width, -1);
        full_map_msg->info.origin.position.x = grid_origin_x;
        full_map_msg->info.origin.position.y = grid_origin_y;
        
        // Reset log odds map
        for(int i = 0; i < HEIGHT*WIDTH; i++) {
            log_odds_map[i] = log_odds_unknown;
        }
    
        RCLCPP_INFO(this->get_logger(), "relocated map");
        get_points(mask_image_msg, depth_image_msg, camera_info_msg);
        map_pub->publish(*full_map_msg);
    }


    void timer_callback() {
        bool recv = depth_recv;
        recv = recv && mask_recv; 
        recv = recv && camera_info_recv;
        recv = recv && odometry_recv;
        if(recv) {
            get_points(mask_image_msg, depth_image_msg, camera_info_msg);
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
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr instant_map_pub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub;

    // Data containers
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    sensor_msgs::msg::Image::SharedPtr depth_image_msg;
    sensor_msgs::msg::Image::SharedPtr mask_image_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;

    rclcpp::TimerBase::SharedPtr timer;

    double prob_hit, prob_miss, prob_unknown, prob_mark;
    double log_odds_hit, log_odds_miss, log_odds_unknown, log_odds_mark;
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
    std::vector<Point> antipoints;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> instant_map_msg;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> full_map_msg;
    bool sim;
    double log_odds_lower_clamp, log_odds_upper_clamp;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
