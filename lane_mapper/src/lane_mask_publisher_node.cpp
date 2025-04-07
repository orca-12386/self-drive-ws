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
    white_mask_upper(255,255,255)
    {
        RCLCPP_INFO(this->get_logger(), "lane_mask_publisher_node started");

        this->declare_parameter("sim", rclcpp::PARAMETER_BOOL);
        this->declare_parameter("depth_sub_topic", rclcpp::PARAMETER_STRING);
        this->declare_parameter("color_sub_topic", rclcpp::PARAMETER_STRING);

        this->initialise_data();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&LaneMaskPublisherNode::timer_callback, this));
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

    void laneChangeStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->lane_change_status = msg->data;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    void initialise_data() {
        std::string depth_sub_topic = this->get_parameter("depth_sub_topic").as_string();
        std::string color_sub_topic = this->get_parameter("color_sub_topic").as_string();
        sim = this->get_parameter("sim").as_bool();
        log(color_sub_topic);
        log(depth_sub_topic);
        if(sim) {
            white_mask_lower = cv::Scalar(230,230,230);
        } else {
            white_mask_lower = cv::Scalar(200,200,200);
        }
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            color_sub_topic, 10, std::bind(&LaneMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            depth_sub_topic, 10, std::bind(&LaneMaskPublisherNode::depthImageCallback, this, std::placeholders::_1));
        
        white_mask_pub = this->create_publisher<sensor_msgs::msg::Image>("/mask/white", 10);
        yellow_mask_pub = this->create_publisher<sensor_msgs::msg::Image>("/mask/yellow", 10);
        rgb_recv = false;
        depth_recv = false;
        lane_change_status = false;
        int erosion_kernel_size = 3;
        int dilation_kernel_size = 3;
        erosion_kernel = getStructuringElement(cv::MORPH_RECT,
            cv::Size(erosion_kernel_size, erosion_kernel_size),
            cv::Point(-1,-1) );
        dilation_kernel = getStructuringElement(cv::MORPH_RECT,
            cv::Size(dilation_kernel_size, dilation_kernel_size),
            cv::Point(-1,-1) );
    }

    void remove_horizon(cv::Mat& mask, const cv::Mat& depth_image) {
        float value;
        float minvalue;
        int horizon_rows = 0;
        for(int i = 0;i<depth_image.rows;i++) {
            minvalue = 20;
            for(int j = 0;j<depth_image.cols;j++) {
                value = depth_image.at<float>(i, j);
                if((value < minvalue && !cvIsNaN(value) && !cvIsInf(value))) {
                    minvalue = value;
                }
            }
            if(minvalue < 20) {
                break;
            } else {
                // log(std::to_string(minvalue));
                horizon_rows = i;
            }
        }
        horizon_roi = cv::Rect(0, 0, depth_image.cols, horizon_rows);    // x, y, width, height
        horizon_mat = mask(horizon_roi);
        horizon_mat.setTo(cv::Scalar::all(0));
    }

    void interpolate_yellow_mask(cv::Mat& yellow_mask) {
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
            int y_min = 0;

            int x_min = static_cast<int>(x0 + (y_min - y0) * (vx / vy));
            int x_max = static_cast<int>(x0 + (y_max - y0) * (vx / vy));

            cv::line(yellow_mask, cv::Point(x_min, y_min), cv::Point(x_max, y_max), cv::Scalar(255, 255, 255), 3, cv::LINE_AA);
        }

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(20, 10));
        cv::morphologyEx(yellow_mask, yellow_mask, cv::MORPH_CLOSE, kernel);
    }
    
    void get_white_mask(const cv::Mat& rgb_image, cv::Mat& white_mask) {
        cv::inRange(rgb_image, white_mask_lower, white_mask_upper, white_mask);
    }

    void get_yellow_mask(const cv::Mat& rgb_image, cv::Mat& yellow_mask) {
        cv::cvtColor(rgb_image, hsv, cv::COLOR_RGB2HSV);    
        cv::inRange(hsv, yellow_mask_lower, yellow_mask_upper, yellow_mask);
        // interpolate_yellow_mask(yellow_mask);        
    }


    void erode_and_dilate(cv::Mat& mask) {
        cv::erode(mask, mask, erosion_kernel);
        cv::dilate(mask, mask, dilation_kernel);
    }

    void publish_mask(const cv::Mat& mask, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub) {
        mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        mask_pub->publish(*mask_msg);
    }

    void create_and_publish_mask(sensor_msgs::msg::Image::SharedPtr rgb, sensor_msgs::msg::Image::SharedPtr depth) { 

        Timer t = Timer("sensor msg to cv mat");
        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        rgb_image = rgb_image_ptr->image;

        depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        depth_image = depth_image_ptr->image;

        if (rgb_image.empty() || depth_image.empty()) {
            return;
        } 

        // log(rgb->encoding);
        if(!rgb->encoding.compare("bgra8")) {
            cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGRA2BGR);
            cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB);
        }

        get_white_mask(rgb_image, white_mask);
        // erode_and_dilate(white_mask);
        publish_mask(white_mask, white_mask_pub);

        get_yellow_mask(rgb_image, yellow_mask);
        remove_horizon(yellow_mask, depth_image);
        // erode_and_dilate(yellow_mask);
        publish_mask(yellow_mask, yellow_mask_pub);
    }

    void timer_callback() {
        if(rgb_recv && depth_recv) {
            create_and_publish_mask(rgb_image_msg, depth_image_msg);
        } else {
            log("Waiting for subscriptions");
        }
    }

    bool sim;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub, depth_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_change_status_sub;
    bool rgb_recv;
    bool depth_recv;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_mask_pub, yellow_mask_pub;
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg, depth_image_msg;
    bool lane_change_status;
    rclcpp::TimerBase::SharedPtr timer;
    cv::Scalar yellow_mask_upper, yellow_mask_lower;
    cv::Scalar white_mask_upper, white_mask_lower;
    cv_bridge::CvImagePtr rgb_image_ptr, depth_image_ptr;
    cv::Mat rgb_image, depth_image;
    cv::Mat hsv;
    cv::Mat white_mask, yellow_mask;
    cv::Mat horizon_mat;
    cv::Rect horizon_roi;
    sensor_msgs::msg::Image::SharedPtr mask_msg;
    cv::Mat erosion_kernel, dilation_kernel;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}