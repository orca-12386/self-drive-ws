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

class HeightMaskPublisherNode : public rclcpp::Node
{
public:
    HeightMaskPublisherNode() : 
    rclcpp::Node("height_mask_publisher_node")
    {
        RCLCPP_INFO(this->get_logger(), "height_mask_publisher_node started");
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/image_raw", 10, std::bind(&HeightMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
    
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/depth/image_raw", 10, std::bind(&HeightMaskPublisherNode::depthImageCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed_node/stereocamera/camera_info", 10, std::bind(&HeightMaskPublisherNode::cameraInfoCallback, this, std::placeholders::_1));
    
        rgb_recv = false;
        depth_recv = false;
        camera_info_recv = false;

        mask_pub = this->create_publisher<sensor_msgs::msg::Image>("/height_mask", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&HeightMaskPublisherNode::timer_callback, this));
    };

private:
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->rgb_image_msg = msg;
        rgb_recv = true;
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->depth_image_msg = msg;
        depth_recv = true;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->camera_info_msg = msg;
        camera_info_recv = true;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    typedef struct Point {
        double x;
        double y;
        double z;
    } Point;

    Point convert_depth_to_point(double u, double v, const double& depth, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        double fx = camera_info->k[0]; 
        double fy = camera_info->k[4];
        double cx = camera_info->k[2];
        double cy = camera_info->k[5];
        // double u = location.x;
        // double v = location.y;
        double z = depth;
        double x = ((u-cx)*z)/fx;
        double y = ((v-cy)*z)/fy;
        Point p = Point();
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }


    void publish_mask(sensor_msgs::msg::Image::SharedPtr rgb, sensor_msgs::msg::Image::SharedPtr depth, sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {    
        Timer t = Timer("sensor msg to cv mat");
        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        rgb_image = rgb_image_ptr->image;

        depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        depth_image = depth_image_ptr->image;

        if (rgb_image.empty() || depth_image.empty()) {
            return;
        } 

        cv::Mat mask(rgb_image.rows,rgb_image.cols, CV_8U, cv::Scalar(0));
        // cv::Mat gray;
        // cv::cvtColor(rgb_image, gray, cv::COLOR_BGR2GRAY);
        // cv::Mat blurred;
        // cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);
        // cv::Mat edges;
        // cv::Canny(blurred, edges, 50, 150);
        // if(edges.empty()) {
        //     return;
        // }
        // log("edge");
        // std::vector<std::vector<cv::Point>> contours;
        // std::vector<cv::Vec4i> hierarchy;
        // cv::findContours(edges, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        // if(contours.size() == 0) {
        //     return;
        // }
        // for (const auto& contour : contours) {
        //     std::vector<cv::Point> approx;
        //     cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);    
        //     if (approx.size() == 4) {
        //         cv::Rect boundingBox = cv::boundingRect(approx);
        //         cv::rectangle(mask, boundingBox, cv::Scalar(255), 2);
        //     }
        // }
        // log("contour");

        Point p;
        double depthvalue;
        for(int i = 0;i<mask.rows;i++) {
            for(int j = 0;j<mask.cols;j++) {
                depthvalue = static_cast<double>(depth_image.at<float>(i,j));
                p = convert_depth_to_point(j, i, depthvalue, camera_info);
                if(!(p.z < 20 && p.y > 0)) {
                    mask.at<int>(i,j) = 0;
                } else {
                    mask.at<int>(i,j) = 255;
                }
            }
        }
        // log("xyz");

        mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        mask_pub->publish(*mask_msg);
        log("published");
    }

    void timer_callback() {
        bool recv;
        recv = rgb_recv;
        recv = recv && depth_recv;
        recv = recv && camera_info_recv;
        if(recv) {
            publish_mask(rgb_image_msg, depth_image_msg, camera_info_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub, depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    
    bool rgb_recv;
    bool depth_recv;
    bool camera_info_recv;
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub;
    
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg, depth_image_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;

    sensor_msgs::msg::Image::SharedPtr mask_msg;

    rclcpp::TimerBase::SharedPtr timer;
    
    cv_bridge::CvImagePtr rgb_image_ptr, depth_image_ptr;
    cv::Mat rgb_image, depth_image;
    cv::Mat y_image, height_mask;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
