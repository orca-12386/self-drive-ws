#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

struct BarrelDetection {
    cv::Point2f centroid;
    double area;
    cv::Point3f camera_point;
    cv::Point3f odom_point;
    double distance_to_robot;
};

class BarrelDetectionNode : public rclcpp::Node
{
public:
    BarrelDetectionNode() : rclcpp::Node("barrel_detection_node") {
        RCLCPP_INFO(this->get_logger(), "Barrel Detection Node started");

        // Declare parameters
        this->declare_parameter<std::string>("color_sub_topic", "/zed/zed_node/rgb/image_rect_color");
        this->declare_parameter<std::string>("depth_sub_topic", "/zed/zed_node/depth/depth_registered");
        this->declare_parameter<std::string>("camera_info_sub_topic", "/zed/zed_node/rgb/camera_info");
        this->declare_parameter<std::string>("barrel_point_pub_topic", "/detector/traffic_drum/coordinates");
        this->declare_parameter<std::string>("mask_pub_topic", "/detector/traffic_drum/mask");
        
        // HSV filter parameters
        this->declare_parameter<int>("hsv_h_min", 0);
        this->declare_parameter<int>("hsv_s_min", 160);
        this->declare_parameter<int>("hsv_v_min", 0);
        this->declare_parameter<int>("hsv_h_max", 25);
        this->declare_parameter<int>("hsv_s_max", 255);
        this->declare_parameter<int>("hsv_v_max", 120);
        
        // Morphological operations parameters
        this->declare_parameter<int>("erode_kernel_size", 3);
        this->declare_parameter<int>("dilate_kernel_size", 5);
        this->declare_parameter<int>("erode_iterations", 1);
        this->declare_parameter<int>("dilate_iterations", 2);
        
        // Detection parameters
        this->declare_parameter<double>("min_area", 100.0);
        this->declare_parameter<double>("max_distance", 6.0);
        
        // Get parameter values
        std::string color_sub_topic = this->get_parameter("color_sub_topic").as_string();
        std::string depth_sub_topic = this->get_parameter("depth_sub_topic").as_string();
        std::string camera_info_sub_topic = this->get_parameter("camera_info_sub_topic").as_string();
        std::string barrel_point_pub_topic = this->get_parameter("barrel_point_pub_topic").as_string();
        std::string mask_pub_topic = this->get_parameter("mask_pub_topic").as_string();
        
        updateHSVParameters();
        updateMorphParameters();
        updateDetectionParameters();

        // Initialize subscriptions
        rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            color_sub_topic, 10, 
            std::bind(&BarrelDetectionNode::rgbCallback, this, std::placeholders::_1));

        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_sub_topic, 10, 
            std::bind(&BarrelDetectionNode::depthCallback, this, std::placeholders::_1));

        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_sub_topic, 10, 
            std::bind(&BarrelDetectionNode::cameraInfoCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&BarrelDetectionNode::odomCallback, this, std::placeholders::_1));

        // Initialize publishers
        barrel_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            barrel_point_pub_topic, 10);
        
        mask_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            mask_pub_topic, 10);

        // Initialize TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize flags
        rgb_received_ = false;
        depth_received_ = false;
        camera_info_received_ = false;
        odom_received_ = false;

        RCLCPP_INFO(this->get_logger(), "Subscribed to topics and ready for detection");
    }

private:
    void updateHSVParameters() {
        hsv_lower_ = cv::Scalar(
            this->get_parameter("hsv_h_min").as_int(),
            this->get_parameter("hsv_s_min").as_int(),
            this->get_parameter("hsv_v_min").as_int()
        );
        hsv_upper_ = cv::Scalar(
            this->get_parameter("hsv_h_max").as_int(),
            this->get_parameter("hsv_s_max").as_int(),
            this->get_parameter("hsv_v_max").as_int()
        );
    }

    void updateMorphParameters() {
        int erode_size = this->get_parameter("erode_kernel_size").as_int();
        int dilate_size = this->get_parameter("dilate_kernel_size").as_int();
        
        erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
            cv::Size(erode_size, erode_size));
        dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
            cv::Size(dilate_size, dilate_size));
            
        erode_iterations_ = this->get_parameter("erode_iterations").as_int();
        dilate_iterations_ = this->get_parameter("dilate_iterations").as_int();
    }

    void updateDetectionParameters() {
        min_area_ = this->get_parameter("min_area").as_double();
        max_distance_ = this->get_parameter("max_distance").as_double();
    }

    void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        rgb_msg_ = msg;
        rgb_received_ = true;
        processImages();
    }

    void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        depth_msg_ = msg;
        depth_received_ = true;
        processImages();
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        camera_info_msg_ = msg;
        camera_info_received_ = true;
        
        // Extract camera intrinsics
        fx_ = camera_info_msg_->k[0];
        fy_ = camera_info_msg_->k[4];
        cx_ = camera_info_msg_->k[2];
        cy_ = camera_info_msg_->k[5];
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_msg_ = msg;
        odom_received_ = true;
        
        // Store robot position
        robot_x_ = odom_msg_->pose.pose.position.x;
        robot_y_ = odom_msg_->pose.pose.position.y;
        robot_z_ = odom_msg_->pose.pose.position.z;
    }

    void processImages() {
        // Check if all required data is available
        if (!rgb_received_ || !depth_received_ || !camera_info_received_ || !odom_received_) {
            return;
        }

        try {
            // Convert ROS images to OpenCV
            auto rgb_cv_ptr = cv_bridge::toCvCopy(rgb_msg_, sensor_msgs::image_encodings::BGR8);
            auto depth_cv_ptr = cv_bridge::toCvCopy(depth_msg_, sensor_msgs::image_encodings::TYPE_32FC1);

            cv::Mat rgb_image = rgb_cv_ptr->image;
            cv::Mat depth_image = depth_cv_ptr->image;

            // Validate image dimensions
            if (rgb_image.size() != depth_image.size()) {
                RCLCPP_WARN(this->get_logger(), "RGB and depth images have different dimensions");
                return;
            }

            // Create HSV mask
            cv::Mat mask = createHSVMask(rgb_image);

            // Apply morphological operations
            cv::Mat processed_mask = applyMorphologicalOps(mask);

            // Get transform from camera frame to odom frame
            geometry_msgs::msg::TransformStamped transform_stamped;
            try {
                transform_stamped = tf_buffer_->lookupTransform(
                    "odom", 
                    depth_msg_->header.frame_id, 
                    tf2::TimePointZero
                );
            } catch (tf2::TransformException& ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
                return;
            }

            // Find barrel detections using connected components
            std::vector<BarrelDetection> detections = findBarrelClusters(processed_mask, depth_image, transform_stamped);

            // Find closest valid barrel
            publishClosestBarrel(detections);
            
            // Publish the processed mask for visualization
            publishMask(processed_mask);

        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception: %s", e.what());
        } catch (cv::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
        }
    }

    cv::Mat createHSVMask(const cv::Mat& rgb_image) {
        cv::Mat hsv_image, mask;
        
        // Convert BGR to HSV
        cv::cvtColor(rgb_image, hsv_image, cv::COLOR_BGR2HSV);
        
        // Create mask based on HSV range
        cv::inRange(hsv_image, hsv_lower_, hsv_upper_, mask);
        
        return mask;
    }

    cv::Mat applyMorphologicalOps(const cv::Mat& mask) {
        cv::Mat processed_mask;
        
        // Erode to remove noise
        cv::erode(mask, processed_mask, erode_kernel_, cv::Point(-1, -1), erode_iterations_);
        
        // Dilate to fill gaps
        cv::dilate(processed_mask, processed_mask, dilate_kernel_, cv::Point(-1, -1), dilate_iterations_);
        
        return processed_mask;
    }

    std::vector<BarrelDetection> findBarrelClusters(const cv::Mat& mask, const cv::Mat& depth_image, 
                                                     const geometry_msgs::msg::TransformStamped& transform_stamped) {
        std::vector<BarrelDetection> detections;

        // Use connected components to find clusters
        cv::Mat labels, stats, centroids;
        int num_components = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

        // Process each component (skip background component 0)
        for (int i = 1; i < num_components; i++) {
            double area = stats.at<int>(i, cv::CC_STAT_AREA);
            
            // Filter by minimum area
            if (area < min_area_) {
                continue;
            }

            // Get centroid
            cv::Point2f centroid(centroids.at<double>(i, 0), centroids.at<double>(i, 1));

            // Get depth at centroid
            int cx = static_cast<int>(std::round(centroid.x));
            int cy = static_cast<int>(std::round(centroid.y));
            
            // Ensure centroid is within image bounds
            if (cx < 0 || cx >= depth_image.cols || cy < 0 || cy >= depth_image.rows) {
                continue;
            }

            float depth = depth_image.at<float>(cy, cx);
            
            // Skip invalid depth readings
            if (std::isnan(depth) || std::isinf(depth) || depth <= 0) {
                continue;
            }

            // Convert to 3D camera coordinates
            cv::Point3f camera_point = convertDepthToPoint(centroid, depth);
            
            // Transform to odom frame
            cv::Point3f odom_point;
            if (!transformToOdomFrame(camera_point, odom_point, transform_stamped)) {
                continue;
            }
            
            // Calculate distance to robot in odom frame
            double distance = calculateDistanceToRobot(odom_point);
            
            // Filter by maximum distance
            if (distance > max_distance_) {
                continue;
            }

            // Create detection
            BarrelDetection detection;
            detection.centroid = centroid;
            detection.area = area;
            detection.camera_point = camera_point;
            detection.odom_point = odom_point;
            detection.distance_to_robot = distance;
            
            detections.push_back(detection);
        }

        return detections;
    }

    cv::Point3f convertDepthToPoint(const cv::Point2f& pixel, float depth) {
        // Convert pixel coordinates to 3D camera coordinates using camera intrinsics
        double fx = camera_info_msg_->k[0]; 
        double fy = camera_info_msg_->k[4];
        double cx = camera_info_msg_->k[2];
        double cy = camera_info_msg_->k[5];
        
        double u = pixel.x;
        double v = pixel.y;
        double z = depth;
        double x = ((u - cx) * z) / fx;
        double y = ((v - cy) * z) / fy;
        
        return cv::Point3f(x, y, z);
    }

    bool transformToOdomFrame(const cv::Point3f& camera_point, cv::Point3f& odom_point, 
                             const geometry_msgs::msg::TransformStamped& transform_stamped) {
        // Create point stamped message in camera frame
        geometry_msgs::msg::PointStamped point_in, point_out;
        point_in.header.frame_id = depth_msg_->header.frame_id;
        point_in.header.stamp = depth_msg_->header.stamp;
        point_in.point.x = camera_point.x;
        point_in.point.y = camera_point.y;
        point_in.point.z = camera_point.z;
        
        try {
            // Transform to odom frame
            tf2::doTransform(point_in, point_out, transform_stamped);
            odom_point.x = point_out.point.x;
            odom_point.y = point_out.point.y;
            odom_point.z = point_out.point.z;
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
            return false;
        }
    }

    double calculateDistanceToRobot(const cv::Point3f& odom_point) {
        // Calculate 3D distance from robot position in odom frame
        double dx = odom_point.x - robot_x_;
        double dy = odom_point.y - robot_y_;
        double dz = odom_point.z - robot_z_;
        
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    void publishClosestBarrel(const std::vector<BarrelDetection>& detections) {
        if (detections.empty()) {
            return;
        }

        // Find the closest detection
        auto closest_detection = *std::min_element(detections.begin(), detections.end(),
            [](const BarrelDetection& a, const BarrelDetection& b) {
                return a.distance_to_robot < b.distance_to_robot;
            });

        // Create and publish point message in odom frame
        geometry_msgs::msg::Point point_msg;
        point_msg.x = closest_detection.odom_point.x;
        point_msg.y = closest_detection.odom_point.y;
        point_msg.z = closest_detection.odom_point.z;

        barrel_point_pub_->publish(point_msg);

        RCLCPP_INFO(this->get_logger(), 
            "Published closest barrel in odom frame at distance %.2f m: (%.2f, %.2f, %.2f)",
            closest_detection.distance_to_robot,
            point_msg.x, point_msg.y, point_msg.z);
    }

    void publishMask(const cv::Mat& mask) {
        try {
            // Convert OpenCV mask to ROS message
            cv_bridge::CvImage cv_image;
            cv_image.header.stamp = this->now();
            cv_image.header.frame_id = rgb_msg_->header.frame_id;
            cv_image.encoding = sensor_msgs::image_encodings::MONO8;
            cv_image.image = mask;
            
            sensor_msgs::msg::Image::SharedPtr mask_msg = cv_image.toImageMsg();
            mask_pub_->publish(*mask_msg);
            
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to publish mask: %s", e.what());
        }
    }

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr barrel_point_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;

    // Message storage
    sensor_msgs::msg::Image::SharedPtr rgb_msg_;
    sensor_msgs::msg::Image::SharedPtr depth_msg_;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg_;
    nav_msgs::msg::Odometry::SharedPtr odom_msg_;

    // Flags
    bool rgb_received_;
    bool depth_received_;
    bool camera_info_received_;
    bool odom_received_;

    // Camera intrinsics
    double fx_, fy_, cx_, cy_;

    // Robot position in odom frame
    double robot_x_, robot_y_, robot_z_;

    // TF2 for coordinate transformations
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // HSV filter parameters
    cv::Scalar hsv_lower_;
    cv::Scalar hsv_upper_;

    // Morphological operation parameters
    cv::Mat erode_kernel_;
    cv::Mat dilate_kernel_;
    int erode_iterations_;
    int dilate_iterations_;

    // Detection parameters
    double min_area_;
    double max_distance_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BarrelDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}