#define HEIGHT 3000
#define WIDTH 3000
#define RESOLUTION 0.08

#define DEBUG

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
        // Initialise subscriptions
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/depth/image_raw", 10, std::bind(&LaneMapperNode::depthImageCallback, this, std::placeholders::_1));

        mask_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/mask", 10, std::bind(&LaneMapperNode::maskImageCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed_node/stereocamera/camera_info", 10, std::bind(&LaneMapperNode::cameraInfoCallback, this, std::placeholders::_1));

        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&LaneMapperNode::odomCallback, this, std::placeholders::_1));


        // Initialise subscription flags
        depth_recv = false;
        mask_recv = false;
        camera_info_recv = false;
        odometry_recv = false;

        // Initialise publishers
        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

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

        for(int i = 0;i<grid_height ;i++) {
            for(int j = 0;j<grid_width;j++) {
                log_odds_map[(i*grid_width) + j] = log_odds_unknown;
            }
        }

        instant_map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        instant_map_msg->data.resize(grid_height*grid_width, 0);
        instant_map_msg->header.frame_id = "map";
        instant_map_msg->info.width = grid_width;
        instant_map_msg->info.height = grid_height;
        instant_map_msg->info.resolution = grid_resolution;
        instant_map_msg->info.origin.position.x = grid_origin_x;
        instant_map_msg->info.origin.position.y = grid_origin_y;
        instant_map_msg->info.origin.position.z = 0.0;
        instant_map_msg->info.origin.orientation.w = 1.0;

        full_map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        full_map_msg->data.resize(grid_height*grid_width, -1);
        full_map_msg->header.frame_id = "map";
        full_map_msg->info.width = grid_width;
        full_map_msg->info.height = grid_height;
        full_map_msg->info.resolution = grid_resolution;
        full_map_msg->info.origin.position.x = grid_origin_x;
        full_map_msg->info.origin.position.y = grid_origin_y;
        full_map_msg->info.origin.position.z = 0.0;
        full_map_msg->info.origin.orientation.w = 1.0;

        timer = this->create_wall_timer(
            std::chrono::milliseconds(50), std::bind(&LaneMapperNode::timer_callback, this));
        
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

    void log_matrix(cv::Mat& mat) {
        float a = mat.at<float>(0, mat.cols-1);
        float b = mat.at<float>(1, mat.cols-1);
        log_debug(std::string("Matrix: ")+std::to_string(a)+std::string(", ")+std::to_string(b));
    }    

    // Points

    typedef struct Point {
        float x;
        float y;
        float z;
        float local_grid_x;
        float local_grid_y;
        int global_grid_x;
        int global_grid_y;
    } Point;

    Point convert_depth_to_point(const cv::Point& location, const float& depth, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        float fx = camera_info->k[0]; 
        float fy = camera_info->k[4];
        float cx = camera_info->k[2];
        float cy = camera_info->k[5];
        float u = location.x;
        float v = location.y;
        float z = depth;
        float x = ((u-cx)*z)/fx;
        float y = ((v-cy)*z)/fy;
        Point p = Point();
        p.x = x;
        p.y = y;
        p.z = z;
        //log_debug(std::string("convert_depth_to_point: ")+std::to_string(p.x)+std::string(",")+std::to_string(p.z));
        return p;
    }

    cv::Mat mask_to_grid(cv::Mat input_matrix, cv::Mat depth_diagonal_matrix, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info, double yaw, float shifted_origin_x, float shifted_origin_y) {
        float fx = camera_info->k[0]; 
        float fy = camera_info->k[4];
        float cx = camera_info->k[2];
        float cy = camera_info->k[5];
        float kinv[6] = {1/fx, -cx/fx, 0, 1};
        cv::Mat camera_info_mat(2, 2, CV_32FC1, kinv);
        float rot[6] = {static_cast<float>(-cos(yaw)), static_cast<float>(sin(yaw)), static_cast<float>(sin(yaw)), static_cast<float>(cos(yaw))};
        cv::Mat rotation_mat(2,2,CV_32FC1,rot);

        log_matrix(input_matrix);
        cv::Mat camera_transformed = camera_info_mat*input_matrix;
        log_matrix(camera_transformed);
        cv::Mat resolution_transformed = camera_transformed/grid_resolution;
        log_matrix(resolution_transformed);
        cv::Mat rotate_transformed = rotation_mat*resolution_transformed;
        log(std::string("rotate"));
        log_matrix(rotate_transformed);

        std::vector<float> row(rotate_transformed.cols, 0);
        row[rotate_transformed.cols-1] = 1;
        cv::Mat row_mat(1, rotate_transformed.cols, CV_32FC1, row.data());
        rotate_transformed.push_back(row_mat);

        float shift[9] = {1,0,shifted_origin_x, 0,1,shifted_origin_y, 0,0,1};
        cv::Mat shift_mat(3,3,CV_32FC1,shift);

        cv::Mat out = shift_mat*rotate_transformed;
        log(std::string("shift"));
        log_matrix(out);

        return out;
    }

    void convert_to_grid_coords(Point& p, float refx, float refy) {
        p.local_grid_x = (p.x - refx)/grid_resolution;
        p.local_grid_y = (p.z - refy)/grid_resolution; 
    }

    void rotate_local_grid(Point& p, double yaw) {
        float original_y = p.local_grid_y;
        p.local_grid_y = (p.local_grid_y*sin(yaw)) - (p.local_grid_x*cos(yaw)); 
        p.local_grid_x = (p.local_grid_x*sin(yaw)) + (original_y*cos(yaw));
    }

    void get_global_grid_coords(Point& p, float shifted_origin_x, float shifted_origin_y) {
        p.global_grid_x = std::lround(p.local_grid_x + shifted_origin_x);
        p.global_grid_y = std::lround(p.local_grid_y + shifted_origin_y);
    }

    float log_odds_to_prob(const float& log_odds) {
        return 1-(1/(1+exp(log_odds)));
    }

    float prob_to_log_odds(const float& prob) {
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

        // log_debug(t.log());

        Timer t3 = Timer("Locations");
        // Mask depth image and convert to point format
        locations.clear();
        cv::findNonZero(mask_image, locations); // Get all nonzero pixel locations
        // log_debug(std::string("Number of non-zero pixel locations: ")+std::to_string(static_cast<int>(locations.size())));
       
        if(locations.size() == 0) {
            return;
        }
        // log_debug(t3.log());

        Timer t4 = Timer("Point vector");

        std::vector<float> depth_diagonal_vec;
        std::vector<float> input_vec;
        float depth_value;
        for (const auto& pt : locations) {
            depth_value = depth_image.at<float>(pt.y, pt.x); // Access color pixel
            if(depth_value<z_threshold) {
                input_vec.push_back(static_cast<float>(pt.x));
                depth_diagonal_vec.push_back(depth_value);
            }
        }
        if(input_vec.size() == 0) {
            return;
        }
        input_vec.insert(input_vec.end(), input_vec.size(), 1);
        cv::Mat input_mat(2, input_vec.size(), CV_32FC1, input_vec.data());
        float* depth_mat_arr = new float[input_vec.size()*input_vec.size()];
        std::fill(depth_mat_arr, depth_mat_arr+(input_vec.size()*input_vec.size())-1, 0);
        for(size_t i = 0;i<depth_diagonal_vec.size();i++) {
            depth_mat_arr[(i*depth_diagonal_vec.size())+i] = depth_diagonal_vec[i];
        }
        cv::Mat depth_diagonal_mat(depth_diagonal_vec.size(), depth_diagonal_vec.size(), CV_32FC1, depth_mat_arr);
        
        // log_debug(t4.log());

        Timer t5 = Timer("Odometry");

        Point odom = Point();
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

        //log_debug("Calculated yaw");
        cv::Mat output_mat = mask_to_grid(input_mat, depth_diagonal_mat, camera_info, yaw, odom.global_grid_x, odom.global_grid_y);

        // log_debug(t5.log());

        Timer t6 = Timer("grid conversion");
        
        std::fill(instant_map_msg->data.begin(), instant_map_msg->data.end(), 0);

        //log_debug("Created map message");
        // log_debug("x");

        

        int min_grid_x = grid_width, max_grid_x = 0;
        int min_grid_y = grid_height, max_grid_y = 0;
        Point it = Point();
        for (size_t i = 0;i < output_mat.cols ; i++) {
            // log_debug(std::to_string(output_mat.at<float>(0, i))+std::string(", ")+std::to_string(output_mat.at<float>(1, i)));
            it.global_grid_x = std::lround(output_mat.at<float>(0, i));
            it.global_grid_y = std::lround(output_mat.at<float>(1, i));
            // log_debug(std::to_string(it.global_grid_x)+std::string(", ")+std::to_string(it.global_grid_y));
            // log_debug("y");
            //log_debug(std::string("iterator:")+std::to_string(it.x)+std::string(",")+std::to_string(it.y));
            // convert_to_grid_coords(*it, 0, 0);
            // log_debug("a");
            // rotate_local_grid(*it, yaw);
            // log_debug("b");
            // get_global_grid_coords(*it, odom.global_grid_x, odom.global_grid_y);
            // log_debug("c");
            // log_debug(std::to_string(it.global_grid_x) + std::string(",") + std::to_string(it.global_grid_y));
            // log_debug(std::to_string((it.global_grid_y*grid_width)+it.global_grid_x));
            if(it.global_grid_y < grid_height && it.global_grid_y >= 0 && it.global_grid_x >= 0 && it.global_grid_x < grid_width) {
                if(it.global_grid_x < min_grid_x) {
                    min_grid_x = it.global_grid_x;
                }
                if(it.global_grid_x > max_grid_x) {
                    max_grid_x = it.global_grid_x;
                }
                if(it.global_grid_y < min_grid_y) {
                    min_grid_y = it.global_grid_y;
                }
                if(it.global_grid_y > max_grid_y) {
                    max_grid_y = it.global_grid_y;
                }
                size_t index = (it.global_grid_y*grid_width)+it.global_grid_x;
                instant_map_msg->data[index] = 100;
                log_odds_map[index] += log_odds_hit-log_odds_miss;
            } else {
                // log("Assignment exceeds map dimensions: "+std::to_string(it.global_grid_x)+std::string(", ")+std::to_string(it.global_grid_y));
                // log_debug(std::to_string(output_mat.rows)+std::string(", ")+std::to_string(output_mat.cols));
            }
            // log_debug("d");
        }

        // log_debug(t6.log());

        for(int i = min_grid_y; i<max_grid_y ;i++) {
            for(int j = min_grid_x;j<max_grid_x;j++) {
                size_t index = (i*grid_width)+j;
                log_odds_map[index] += log_odds_miss;
                if(log_odds_map[index] >= log_odds_mark) {
                    full_map_msg->data[index] = 100;
                } else if(log_odds_map[index] > log_odds_miss) {
                    full_map_msg->data[index] = -1;
                } else {
                    full_map_msg->data[index] = 0;
                }
            }
        }

        Timer t7 = Timer("create and publish");

        full_map_msg->header.stamp = this->now();
        //log_debug("Created map msg");

        //publish map
        map_pub->publish(*full_map_msg);

        // log_debug(t7.log());

        delete depth_mat_arr;

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

    float prob_hit, prob_miss, prob_unknown, prob_mark;
    float log_odds_hit, log_odds_miss, log_odds_unknown, log_odds_mark;
    float log_odds_map[HEIGHT*WIDTH];
    
    int grid_height;
    int grid_width;
    float grid_resolution;
    float grid_origin_x;
    float grid_origin_y;

    int y_threshold, z_threshold;

    cv_bridge::CvImagePtr depth_image_ptr, mask_image_ptr;
    cv::Mat depth_image, mask_image;
    std::vector<cv::Point> locations;
    std::vector<Point> points;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> instant_map_msg;
    std::shared_ptr<nav_msgs::msg::OccupancyGrid> full_map_msg;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}