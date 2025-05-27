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
        return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());
    }

    std::string log() {
        return name + ": " + std::to_string(this->elapsed()) + " ms";
    }

private:
    std::chrono::steady_clock::time_point begin, end;
    std::string name;
};

class LaneMaskPublisherNode : public rclcpp::Node
{
public:
    LaneMaskPublisherNode() : rclcpp::Node("lane_mask_publisher_node")
    {
        RCLCPP_INFO(this->get_logger(), "lane_mask_publisher_node started");

        declare_parameters();
        load_parameters();
        setup_subscriptions();
        setup_publishers();
        setup_kernels();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&LaneMaskPublisherNode::timer_callback, this));
    };

private:
    // Parameters
    bool sim;
    float target_v;
    cv::Scalar yellow_mask_upper, yellow_mask_lower;
    cv::Scalar white_mask_rgb_lower, white_mask_rgb_upper;
    cv::Scalar white_mask_hsv_lower, white_mask_hsv_upper;

    // ROS
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub, depth_sub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr white_mask_pub, yellow_mask_pub;
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg, depth_image_msg;
    bool rgb_recv = false, depth_recv = false;
    rclcpp::TimerBase::SharedPtr timer;

    // OpenCV
    cv_bridge::CvImagePtr rgb_image_ptr, depth_image_ptr;
    cv::Mat rgb_image, depth_image;
    cv::Mat hsv, white_mask, yellow_mask;
    cv::Mat erosion_kernel, dilation_kernel;
    cv::Rect horizon_roi;
    sensor_msgs::msg::Image::SharedPtr mask_msg;

    void declare_parameters() {
        this->declare_parameter("sim", false);
        this->declare_parameter("depth_sub_topic", "/zed_node/depth/depth_registered");
        this->declare_parameter("color_sub_topic", "/zed_node/rgb/image_rect_color");
        
    }

    void load_parameters() {
        sim = this->get_parameter("sim").as_bool();

        // Hardcoded mask ranges depending on sim mode


        if (sim) {
            white_mask_rgb_lower = cv::Scalar(230, 230, 230);
            white_mask_rgb_upper = cv::Scalar(255, 255, 255);
            yellow_mask_lower = cv::Scalar(0, 69, 41);
            yellow_mask_upper = cv::Scalar(179, 255, 255);
            target_v = -1;  // No brightness adjustment
        } else {
            // Night
            // white_mask_hsv_lower = cv::Scalar(0, 0, 180);
            // white_mask_hsv_upper = cv::Scalar(180, 80, 255);
            // yellow_mask_lower = cv::Scalar(0, 69, 41);
            // yellow_mask_upper = cv::Scalar(179, 255, 255);

            // Evening
            // white_mask_hsv_lower = cv::Scalar(0, 0, 215);
            // white_mask_hsv_upper = cv::Scalar(85, 10, 255);
            // yellow_mask_lower = cv::Scalar(50, 0, 230); 
            // yellow_mask_upper = cv::Scalar(180, 255, 255);

            // Late Evening
            // white_mask_hsv_lower = cv::Scalar(0, 0, 230);
            // white_mask_hsv_upper = cv::Scalar(180, 40, 255);
            // yellow_mask_lower = cv::Scalar(62, 94, 106);
            // yellow_mask_upper = cv::Scalar(113, 255, 255);

            // Day
            // white_mask_hsv_lower = cv::Scalar(0, 0, 255);
            // white_mask_hsv_upper = cv::Scalar(77, 30, 255); //34
            // yellow_mask_lower = cv::Scalar(50, 0, 230); 
            // yellow_mask_upper = cv::Scalar(180, 255, 255);

            // Inside workshop
            white_mask_hsv_lower = cv::Scalar(0, 0, 255);
            white_mask_hsv_upper = cv::Scalar(77, 30, 255); //34
            yellow_mask_lower = cv::Scalar(26, 68, 94); 
            yellow_mask_upper = cv::Scalar(179, 255, 255);
            
            target_v = 140; // Target brightness value
        }
    }

    void setup_subscriptions() {
        auto color_sub_topic = this->get_parameter("color_sub_topic").as_string();
        auto depth_sub_topic = this->get_parameter("depth_sub_topic").as_string();

        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            color_sub_topic, 10, std::bind(&LaneMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            depth_sub_topic, 10, std::bind(&LaneMaskPublisherNode::depthImageCallback, this, std::placeholders::_1));
    }

    void setup_publishers() {
        white_mask_pub = this->create_publisher<sensor_msgs::msg::Image>("/mask/white", 10);
        yellow_mask_pub = this->create_publisher<sensor_msgs::msg::Image>("/mask/yellow", 10);
    }

    void setup_kernels() {
        int erosion_kernel_size;
        int dilation_kernel_size;
        if(this->sim) {
            erosion_kernel_size = 3;
            dilation_kernel_size = 3;
        } else {
            erosion_kernel_size = 5;
            dilation_kernel_size = 7;    
        }
        erosion_kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(erosion_kernel_size, erosion_kernel_size));
        dilation_kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(dilation_kernel_size, dilation_kernel_size));
    }

    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        rgb_image_msg = msg;
        rgb_recv = true;
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        depth_image_msg = msg;
        depth_recv = true;
    }

    void timer_callback() {
        if (rgb_recv && depth_recv) {
            create_and_publish_mask(rgb_image_msg, depth_image_msg);
        }
    }

    void create_and_publish_mask(const sensor_msgs::msg::Image::SharedPtr& rgb, const sensor_msgs::msg::Image::SharedPtr& depth) {
        Timer t("sensor msg to cv mat");
        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);

        rgb_image = rgb_image_ptr->image;
        depth_image = depth_image_ptr->image;

        if (rgb_image.empty() || depth_image.empty()) {
            return;
        }

        if (rgb->encoding == "bgra8") {
            cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGRA2BGR);
            cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB);
        }

        generate_white_mask(rgb_image, white_mask);
        remove_horizon(white_mask, depth_image);
        publish_mask(white_mask, white_mask_pub);

        generate_yellow_mask(rgb_image, yellow_mask);
        remove_horizon(yellow_mask, depth_image);
        publish_mask(yellow_mask, yellow_mask_pub);
    }

    void generate_white_mask(const cv::Mat& image, cv::Mat& mask) {
        if (sim) {
            cv::inRange(image, white_mask_rgb_lower, white_mask_rgb_upper, mask);
        } else {
            cv::Mat bright_image = adjust_brightness(image, target_v);
            cv::cvtColor(bright_image, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, white_mask_hsv_lower, white_mask_hsv_upper, mask);
            apply_morphology(mask);
        }
    }

    void generate_yellow_mask(const cv::Mat& image, cv::Mat& mask) {
            cv::Mat bright_image = adjust_brightness(image, target_v);
            cv::cvtColor(bright_image, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, yellow_mask_lower, yellow_mask_upper, mask);
            apply_morphology(mask);
    }

    void remove_horizon(cv::Mat& mask, const cv::Mat& depth_image) {
        int horizon_rows = 0;
        for (int i = 0; i < depth_image.rows; ++i) {
            double min_val;
            cv::minMaxLoc(depth_image.row(i), &min_val);
            if (min_val < 20) {
                break;
            } else {
                horizon_rows = i;
            }
        }
        if (horizon_rows > 0) {
            horizon_roi = cv::Rect(0, 0, depth_image.cols, horizon_rows);
            mask(horizon_roi).setTo(0);
        }
    }

    void apply_morphology(cv::Mat& mask) {
        cv::erode(mask, mask, erosion_kernel);
        cv::dilate(mask, mask, dilation_kernel);
    }

    cv::Mat adjust_brightness(const cv::Mat& image, float target_v) {
        if (target_v < 0) {
            return image.clone();
        }

        cv::Mat hsv_image;
        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);
        std::vector<cv::Mat> channels;
        cv::split(hsv_image, channels);

        double avg_v = cv::mean(channels[2])[0];
        double scale_factor = (avg_v > 0) ? (target_v / avg_v) : 1.0;
        channels[2] = cv::min(channels[2] * scale_factor, 255.0);

        cv::merge(channels, hsv_image);
        cv::Mat bright_image;
        cv::cvtColor(hsv_image, bright_image, cv::COLOR_HSV2BGR);
        return bright_image;
    }

    void publish_mask(const cv::Mat& mask, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& pub) {
        auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask).toImageMsg();
        pub->publish(*mask_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneMaskPublisherNode>());
    rclcpp::shutdown();
    return 0;
}
