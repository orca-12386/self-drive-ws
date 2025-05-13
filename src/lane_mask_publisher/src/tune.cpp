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
        if (!sim) {
            init_trackbar_window();
        }

        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&LaneMaskPublisherNode::timer_callback, this));
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
        this->declare_parameter("depth_sub_topic", "/zed/zed_node/depth/depth_registered");
        this->declare_parameter("color_sub_topic", "/zed/zed_node/rgb/image_rect_color");
        
    }
    void init_trackbar_window() {
        cv::namedWindow("White HSV Tuner", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("H low", "White HSV Tuner", &white_h_low, 180);
        cv::createTrackbar("S low", "White HSV Tuner", &white_s_low, 255);
        cv::createTrackbar("V low", "White HSV Tuner", &white_v_low, 255);
        cv::createTrackbar("H high", "White HSV Tuner", &white_h_high, 180);
        cv::createTrackbar("S high", "White HSV Tuner", &white_s_high, 255);
        cv::createTrackbar("V high", "White HSV Tuner", &white_v_high, 255);

        cv::namedWindow("Yellow HSV Tuner", cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("H low", "Yellow HSV Tuner", &yellow_h_low, 180);
        cv::createTrackbar("S low", "Yellow HSV Tuner", &yellow_s_low, 255);
        cv::createTrackbar("V low", "Yellow HSV Tuner", &yellow_v_low, 255);
        cv::createTrackbar("H high", "Yellow HSV Tuner", &yellow_h_high, 180);
        cv::createTrackbar("S high", "Yellow HSV Tuner", &yellow_s_high, 255);
        cv::createTrackbar("V high", "Yellow HSV Tuner", &yellow_v_high, 255);
    }

    int white_h_low = 0, white_s_low = 0, white_v_low = 180;
    int white_h_high = 180, white_s_high = 80, white_v_high = 255;


    int yellow_h_low = 0, yellow_s_low = 0, yellow_v_low = 180;
    int yellow_h_high = 180, yellow_s_high = 80, yellow_v_high = 255;
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
            white_mask_hsv_lower = cv::Scalar(0, 0, 180);
            white_mask_hsv_upper = cv::Scalar(180, 80, 255);
            yellow_mask_lower = cv::Scalar(37,83, 255);
            yellow_mask_upper = cv::Scalar(180, 255, 255);
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
        int kernel_size = 3;
        erosion_kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
        dilation_kernel = getStructuringElement(cv::MORPH_RECT, cv::Size(kernel_size, kernel_size));
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
        if(!rgb_recv)
        RCLCPP_INFO(this->get_logger(), "NO Rgb");
        if(depth_recv)
        RCLCPP_INFO(this->get_logger(), "NO DEPTH");
    }

    void create_and_publish_mask(const sensor_msgs::msg::Image::SharedPtr& rgb, const sensor_msgs::msg::Image::SharedPtr& depth) {
        Timer t("sensor msg to cv mat");
        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);

        rgb_image = rgb_image_ptr->image;
        depth_image = depth_image_ptr->image;

        if (rgb_image.empty() || depth_image.empty()) {
            if(rgb_image.empty())
            RCLCPP_INFO(this->get_logger(), "RGB Image not recieved");
            if(depth_image.empty())
            RCLCPP_INFO(this->get_logger(), "Depth Image not recieved");

            return;
        }

        if (rgb->encoding == "bgra8") {
            cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGRA2BGR);
            cv::cvtColor(rgb_image, rgb_image, cv::COLOR_BGR2RGB);
        }

        generate_white_mask(rgb_image, white_mask);
        publish_mask(white_mask, white_mask_pub);

        generate_yellow_mask(rgb_image, yellow_mask);
        remove_horizon(yellow_mask, depth_image);
        publish_mask(yellow_mask, yellow_mask_pub);
    }

    void generate_white_mask(const cv::Mat& image, cv::Mat& mask) {
        if (sim) {
            cv::inRange(image, white_mask_rgb_lower, white_mask_rgb_upper, mask);
        } else {
            // white_mask_hsv_lower = cv::Scalar(white_h_low, white_s_low, white_v_low);
            // white_mask_hsv_upper = cv::Scalar(white_h_high, white_s_high, white_v_high);

            cv::Mat bright_image = adjust_brightness(image, target_v);
            cv::cvtColor(bright_image, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, white_mask_hsv_lower, white_mask_hsv_upper, mask);
            apply_morphology(mask);

            // Optionally show the image for debugging
            // cv::imshow("White HSV Tuner", mask);
            // cv::waitKey(1);
        }
    }


    void generate_yellow_mask(const cv::Mat& image, cv::Mat& mask) {
        cv::cvtColor(image, hsv, cv::COLOR_RGB2HSV);

        if (sim) {
            cv::inRange(image, yellow_mask_lower, yellow_mask_upper, mask);
        } else {
            yellow_mask_lower = cv::Scalar(yellow_h_low, yellow_s_low, yellow_v_low);
            yellow_mask_upper = cv::Scalar(yellow_h_high, yellow_s_high, yellow_v_high);

            cv::Mat bright_image = adjust_brightness(image, target_v);
            cv::cvtColor(bright_image, hsv, cv::COLOR_BGR2HSV);
            cv::inRange(hsv, yellow_mask_lower, yellow_mask_upper, mask);
            apply_morphology(mask);

            // Optionally show the image for debugging
            cv::imshow("Yellow HSV Tuner", mask);
            cv::waitKey(1);
        }
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
