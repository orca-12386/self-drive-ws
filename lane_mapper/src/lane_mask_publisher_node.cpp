// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>

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

class LaneMaskPublisherNode : public rclcpp::Node
{
public:
    LaneMaskPublisherNode() : 
    rclcpp::Node("lane_mask_publisher_node"), 
    yellow_mask_upper(210,255,255), 
    yellow_mask_lower(0,100,100),
    white_mask_upper(190,190,190),
    white_mask_lower(170,170,170)
    {
        RCLCPP_INFO(this->get_logger(), "lane_mask_publisher_node started");
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/image_raw", 10, std::bind(&LaneMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
    
        lane_change_status_sub = create_subscription<std_msgs::msg::Bool>(
            "/lane_change_status", 10, 
            std::bind(&LaneMaskPublisherNode::laneChangeStatusCallback, this, std::placeholders::_1));

        rgb_recv = false;
        mask_pub = this->create_publisher<sensor_msgs::msg::Image>("/mask", 10);

        lane_change_status = false;
        white_threshold = 150;
        target_v = 140;

        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&LaneMaskPublisherNode::timer_callback, this));
    };

private:
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->rgb_image_msg = msg;
        rgb_recv = true;
    }

    void laneChangeStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->lane_change_status = msg->data;
    }

    void publish_mask(sensor_msgs::msg::Image::SharedPtr rgb) {    
        Timer t = Timer("sensor msg to cv mat");
        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        rgb_image = rgb_image_ptr->image;

        if (rgb_image.empty()) {
            return;
        } 

        cv::cvtColor(rgb_image, hsv, cv::COLOR_RGB2HSV);    

        cv::inRange(rgb_image, white_mask_lower, white_mask_upper, mask);

        if (!lane_change_status) {
            cv::inRange(hsv, yellow_mask_lower, yellow_mask_upper, yellow_mask);
            
            cv::Mat edges;
            cv::Canny(yellow_mask, edges, 50, 150, 3);

            std::vector<cv::Vec4i> lines;
            int rho = 1;
            double theta = CV_PI / 180;
            int threshold = 20;
            int minLineLength = 10;
            int maxLineGap = 300;
            cv::HoughLinesP(edges, lines, rho, theta, threshold, minLineLength, maxLineGap);

            std::vector<cv::Point> lane_points;
            for (const auto& l : lines) {
                lane_points.emplace_back(l[0], l[1]);
                lane_points.emplace_back(l[2], l[3]);
            }

            if (!lane_points.empty()) {
                cv::Vec4f line_params;
                cv::fitLine(lane_points, line_params, cv::DIST_L2, 0, 0.01, 0.01);

                float vx = line_params[0], vy = line_params[1];
                float x0 = line_params[2], y0 = line_params[3];

                int y_max = yellow_mask.rows;
                int y_min = static_cast<int>(y_max * (1.5 / 5.0));

                int x_min = static_cast<int>(x0 + (y_min - y0) * (vx / vy));
                int x_max = static_cast<int>(x0 + (y_max - y0) * (vx / vy));

                cv::line(yellow_mask, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
            }

            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 10));
            cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel);
            cv::bitwise_or(yellow_mask, mask, mask);
        }

        mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        mask_pub->publish(*mask_msg);
    }

    void timer_callback() {
        if(rgb_recv) {
            publish_mask(rgb_image_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_change_status_sub;
    bool rgb_recv;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub;
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg;
    bool lane_change_status;
    rclcpp::TimerBase::SharedPtr timer;
    cv::Scalar yellow_mask_upper, yellow_mask_lower;
    cv::Scalar white_mask_upper, white_mask_lower;
    int white_threshold;
    int target_v;
    cv_bridge::CvImagePtr rgb_image_ptr;
    cv::Mat rgb_image;
    cv::Mat hsv, mask, yellow_mask;
    sensor_msgs::msg::Image::SharedPtr mask_msg;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
