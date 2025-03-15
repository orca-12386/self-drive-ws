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
    LaneMaskPublisherNode() : rclcpp::Node("lane_mask_publisher_node"), yellow_mask_upper(210,255,255), yellow_mask_lower(0,100,100) {
        RCLCPP_INFO(this->get_logger(), "lane_mask_publisher_node started");
        // Initialise subscriptions
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/image_raw", 10, std::bind(&LaneMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
    
        lane_change_status_sub = create_subscription<std_msgs::msg::Bool>(
            "/lane_change_status", 10, 
            std::bind(&LaneMaskPublisherNode::laneChangeStatusCallback, this, std::placeholders::_1));


        // Initialise subscription flags
        rgb_recv = false;

        // Initialise publishers
        mask_pub = this->create_publisher<sensor_msgs::msg::Image>("/mask", 10);

        lane_change_status = false;
        white_threshold = 200;
        target_v = 140;


        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&LaneMaskPublisherNode::timer_callback, this));
        
    };

private:
    // Callback functions
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->rgb_image_msg = msg;
        rgb_recv = true;
    }

    void laneChangeStatusCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        this->lane_change_status = msg->data;
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

    cv::Mat brighten_hsv(const cv::Mat& hsv) {
        std::vector<cv::Mat> channels;
        cv::split(hsv, channels);
        cv::Mat v = channels[2];
        cv::Mat vmeans;
        cv::reduce(v, vmeans, 1, cv::REDUCE_AVG);
        //log_debug("Calculated HSV averages");

        double scale_factor;
        for (int i = 0; i < v.rows; i++) {
            int* v_rowptr = v.ptr<int>(i); // Get pointer to row
            int* vmeans_rowptr = vmeans.ptr<int>(i); // Get pointer to row
            int avg = vmeans_rowptr[0];
            if(avg > 0) {
                scale_factor = 1.0 * target_v / avg;
            } else {
                scale_factor = 1.0;
            }
            for (int j = 0; j < v.cols; j++) {
                int val = scale_factor*v_rowptr[j];          
                if(val > 255) {
                    val = 255;
                } else if (val < 0) {
                    val = 0;
                }
                vmeans_rowptr[j] = val;
            }
        }
        channels[2] = v;
        cv::Mat brightened_hsv_image;
        merge(channels, brightened_hsv_image);
        return brightened_hsv_image;
    }

    void publish_mask(sensor_msgs::msg::Image::SharedPtr rgb) {    

        Timer t = Timer("sensor msg to cv mat");

        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        rgb_image = rgb_image_ptr->image;

        if(rgb_image.empty()) {
            return;
        } 

        log_debug(t.log());

        cv::cvtColor(rgb_image, hsv, cv::COLOR_RGB2HSV);    
        cv::cvtColor(rgb_image, gray, cv::COLOR_RGB2GRAY);
    
        // Timer t2 = Timer("Brightening");
        // cv::Mat brightened_hsv_image = brighten_hsv(hsv);
        // //log_debug("Brightened HSV");
        // cv::Mat brightened_rgb_image;
        // cvtColor(brightened_hsv_image, brightened_rgb_image, cv::COLOR_HSV2RGB);
        // sensor_msgs::msg::Image::SharedPtr brgb_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "rgb8", brightened_rgb_image).toImageMsg();
        // mask_pub->publish(*brgb_msg);
        // //log_debug("Published brightened RGB");
        // log_debug(t2.log());
    
        Timer t3 = Timer("Mask");
        // White mask
        GaussianBlur(gray, mask, cv::Size(5, 5), 0, 0);
        threshold(mask, mask, white_threshold, 255, cv::THRESH_BINARY);
        // Yellow mask
        if(!lane_change_status) {
            cv::inRange(hsv, yellow_mask_lower, yellow_mask_upper, yellow_mask);
            cv::bitwise_or(yellow_mask, mask, mask);    
        }

        log_debug(t3.log());
        // int kernel_size = 7;
        // cv::Mat kernel = getStructuringElement(cv::MORPH_RECT,
        //     cv::Size(kernel_size, kernel_size),
        //     cv::Point(-1,-1) );
        // cv::erode(mask, mask, kernel);
        // cv::dilate(mask, mask, kernel);

        // cv::inRange(rgb_image, cv::Scalar(0, 0, 0), cv::Scalar(150, 150, 150), mask);

        mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        mask_pub->publish(*mask_msg);
        //log_debug("Published mask");
    }

    void timer_callback() {
        bool recv = rgb_recv; 
        if(recv) {
            publish_mask(rgb_image_msg);
        } else {
            log_debug("Waiting for subscriptions");
        }
    }

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr lane_change_status_sub;

    // Subscription flags
    bool rgb_recv;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub;

    // Data containers
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg;

    bool lane_change_status;

    rclcpp::TimerBase::SharedPtr timer;

    cv::Scalar yellow_mask_upper, yellow_mask_lower;
    int white_threshold;
    int target_v;

    cv_bridge::CvImagePtr rgb_image_ptr;
    cv::Mat rgb_image;
    cv::Mat hsv, gray;
    cv::Mat mask, yellow_mask;
    sensor_msgs::msg::Image::SharedPtr mask_msg;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}