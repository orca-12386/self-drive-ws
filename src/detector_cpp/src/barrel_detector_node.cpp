#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>
#include <vector>
#include <algorithm>

class BarrelDetectionNode : public rclcpp::Node
{
public:
    BarrelDetectionNode() : Node("barrel_detection_node")
    {
        // Initialize parameters
        this->declare_parameter("color_sub_topic", "/zed/zed_node/rgb/image_rect_color");
        this->declare_parameter("depth_sub_topic", "/zed/zed_node/depth/depth_registered");
        this->declare_parameter("camera_info_sub_topic", "/zed/zed_node/rgb/camera_info");
        this->declare_parameter("sim", false);
        this->declare_parameter("max_detection_distance", 6.0);
        
        // HSV filter parameters (to be filled later)
        this->declare_parameter("hsv_lower_h", 0);
        this->declare_parameter("hsv_lower_s", 160);
        this->declare_parameter("hsv_lower_v", 0);
        this->declare_parameter("hsv_upper_h", 25);
        this->declare_parameter("hsv_upper_s", 255);
        this->declare_parameter("hsv_upper_v", 125);
        
        // Morphological operations parameters
        this->declare_parameter("erode_kernel_size", 3);
        this->declare_parameter("dilate_kernel_size", 5);
        this->declare_parameter("erode_iterations", 1);
        this->declare_parameter("dilate_iterations", 1);
        
        // Connected components parameters
        this->declare_parameter("min_component_area", 200);
        this->declare_parameter("max_component_area", 10000);

        // Get parameter values
        color_topic_ = this->get_parameter("color_sub_topic").as_string();
        depth_topic_ = this->get_parameter("depth_sub_topic").as_string();
        camera_info_topic_ = this->get_parameter("camera_info_sub_topic").as_string();
        is_sim_ = this->get_parameter("sim").as_bool();
        max_detection_distance_ = this->get_parameter("max_detection_distance").as_double();

        // Initialize HSV thresholds
        updateHSVThresholds();
        
        // Initialize morphological kernels
        updateMorphologyKernels();

        // Initialize TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Initialize subscribers with message filters for synchronization
        color_sub_.subscribe(this, color_topic_);
        depth_sub_.subscribe(this, depth_topic_);
        
        // Approximate time synchronization policy
        sync_policy_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), color_sub_, depth_sub_);
        sync_policy_->registerCallback(
            std::bind(&BarrelDetectionNode::imageCallback, this, 
                     std::placeholders::_1, std::placeholders::_2));

        // Subscribe to camera info and odometry
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 10,
            std::bind(&BarrelDetectionNode::cameraInfoCallback, this, std::placeholders::_1));
            
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&BarrelDetectionNode::odomCallback, this, std::placeholders::_1));

        // Initialize publisher
        barrel_position_pub_ = this->create_publisher<geometry_msgs::msg::Point>(
            "/detector/traffic_drum/coordinates", 10);

        // Initialize member variables
        camera_info_received_ = false;
        bot_position_received_ = false;

        RCLCPP_INFO(this->get_logger(), "Barrel Detection Node initialized");
    }

private:
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image, sensor_msgs::msg::Image>;

    // Subscribers and synchronizer
    message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_policy_;
    
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    // Publisher
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr barrel_position_pub_;

    // TF2
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Parameters
    std::string color_topic_, depth_topic_, camera_info_topic_, depth_frame_id_;
    bool is_sim_;
    double max_detection_distance_;
    
    // HSV thresholds
    cv::Scalar hsv_lower_, hsv_upper_;
    
    // Morphological operation kernels
    cv::Mat erode_kernel_, dilate_kernel_;
    int erode_iterations_, dilate_iterations_;
    
    // Connected components parameters
    int min_component_area_, max_component_area_;

    // Camera intrinsics
    bool camera_info_received_;
    double fx_, fy_, cx_, cy_;
    
    // Robot state
    bool bot_position_received_;
    geometry_msgs::msg::Point bot_position_;
    geometry_msgs::msg::Quaternion bot_orientation_;
    
    // Latest depth timestamp and frame
    rclcpp::Time depth_stamp_;

    void updateHSVThresholds()
    {
        int h_low = this->get_parameter("hsv_lower_h").as_int();
        int s_low = this->get_parameter("hsv_lower_s").as_int();
        int v_low = this->get_parameter("hsv_lower_v").as_int();
        int h_high = this->get_parameter("hsv_upper_h").as_int();
        int s_high = this->get_parameter("hsv_upper_s").as_int();
        int v_high = this->get_parameter("hsv_upper_v").as_int();
        
        hsv_lower_ = cv::Scalar(h_low, s_low, v_low);
        hsv_upper_ = cv::Scalar(h_high, s_high, v_high);
    }

    void updateMorphologyKernels()
    {
        int erode_size = this->get_parameter("erode_kernel_size").as_int();
        int dilate_size = this->get_parameter("dilate_kernel_size").as_int();
        erode_iterations_ = this->get_parameter("erode_iterations").as_int();
        dilate_iterations_ = this->get_parameter("dilate_iterations").as_int();
        min_component_area_ = this->get_parameter("min_component_area").as_int();
        max_component_area_ = this->get_parameter("max_component_area").as_int();
        
        erode_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                 cv::Size(erode_size, erode_size));
        dilate_kernel_ = cv::getStructuringElement(cv::MORPH_ELLIPSE, 
                                                  cv::Size(dilate_size, dilate_size));
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!camera_info_received_)
        {
            fx_ = msg->k[0];  // K[0]
            fy_ = msg->k[4];  // K[4]
            cx_ = msg->k[2];  // K[2]
            cy_ = msg->k[5];  // K[5]
            camera_info_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera intrinsics received");
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        bot_position_ = msg->pose.pose.position;
        bot_orientation_ = msg->pose.pose.orientation;
        bot_position_received_ = true;
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& color_msg,
                      const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        if (!camera_info_received_ || !bot_position_received_)
        {
            return;
        }

        depth_stamp_ = depth_msg->header.stamp;

        try
        {
            // Convert ROS images to OpenCV
            cv_bridge::CvImagePtr color_ptr = cv_bridge::toCvCopy(color_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_32FC1);
            
            cv::Mat color_image = color_ptr->image;
            cv::Mat depth_image = depth_ptr->image;

            depth_frame_id_ = depth_msg->header.frame_id;

            // Process the images
            std::vector<cv::Point2f> barrel_centroids = detectBarrels(color_image);
            
            if (!barrel_centroids.empty())
            {
                geometry_msgs::msg::Point closest_barrel = findClosestBarrel(barrel_centroids, depth_image);
                
                if (closest_barrel.x != 0 || closest_barrel.y != 0 || closest_barrel.z != 0)
                {
                    barrel_position_pub_->publish(closest_barrel);
                }
            }
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    std::vector<cv::Point2f> detectBarrels(const cv::Mat& color_image)
    {
        std::vector<cv::Point2f> centroids;
        
        // Convert to HSV
        cv::Mat hsv_image;
        cv::cvtColor(color_image, hsv_image, cv::COLOR_BGR2HSV);
        
        // Apply HSV mask
        cv::Mat mask;
        cv::inRange(hsv_image, hsv_lower_, hsv_upper_, mask);
        
        // Morphological operations to remove noise
        cv::Mat cleaned_mask;
        cv::erode(mask, cleaned_mask, erode_kernel_, cv::Point(-1, -1), erode_iterations_);
        cv::dilate(cleaned_mask, cleaned_mask, dilate_kernel_, cv::Point(-1, -1), dilate_iterations_);
        
        // Find connected components
        cv::Mat labels, stats, centroids_mat;
        int num_components = cv::connectedComponentsWithStats(cleaned_mask, labels, stats, centroids_mat, 8, CV_32S);
        
        // Process each component (skip background component at index 0)
        for (int i = 1; i < num_components; i++)
        {
            int area = stats.at<int>(i, cv::CC_STAT_AREA);
            
            // Filter components by area
            if (area >= min_component_area_ && area <= max_component_area_)
            {
                double cx = centroids_mat.at<double>(i, 0);
                double cy = centroids_mat.at<double>(i, 1);
                centroids.emplace_back(cx, cy);
            }
        }
        
        return centroids;
    }

    geometry_msgs::msg::Point findClosestBarrel(const std::vector<cv::Point2f>& centroids, 
                                               const cv::Mat& depth_image)
    {
        geometry_msgs::msg::Point closest_barrel;
        double min_distance = std::numeric_limits<double>::max();
        
        for (const auto& centroid : centroids)
        {
            // Convert pixel to 3D point
            geometry_msgs::msg::Point world_point = pixelToWorldPoint(centroid, depth_image);
            
            if (world_point.x == 0 && world_point.y == 0 && world_point.z == 0)
            {
                continue; // Invalid point
            }
            
            // Calculate distance to robot
            double distance = std::sqrt(
                std::pow(world_point.x - bot_position_.x, 2) +
                std::pow(world_point.y - bot_position_.y, 2) +
                std::pow(world_point.z - bot_position_.z, 2)
            );
            
            // Check if within detection range and closer than previous
            if (distance <= max_detection_distance_ && distance < min_distance)
            {
                min_distance = distance;
                closest_barrel = world_point;
            }
        }
        
        return closest_barrel;
    }

    geometry_msgs::msg::Point pixelToWorldPoint(const cv::Point2f& pixel, const cv::Mat& depth_image)
    {
        geometry_msgs::msg::Point world_point;
        
        int x = static_cast<int>(pixel.x);
        int y = static_cast<int>(pixel.y);
        
        // Check bounds
        if (x < 0 || x >= depth_image.cols || y < 0 || y >= depth_image.rows)
        {
            return world_point; // Return zero point
        }
        
        float depth = depth_image.at<float>(y, x);
        
        // Check for valid depth
        if (depth <= 0 || std::isnan(depth) || std::isinf(depth))
        {
            return world_point; // Return zero point
        }
        
        // Convert pixel to camera coordinates
        geometry_msgs::msg::Point camera_point;
        camera_point.x = (pixel.x - cx_) * depth / fx_;
        camera_point.y = (pixel.y - cy_) * depth / fy_;
        camera_point.z = depth;
        
        // Transform to world coordinates using the provided transformation logic
        return transformToWorld(camera_point);
    }

    geometry_msgs::msg::Point transformToWorld(const geometry_msgs::msg::Point& camera_point)
    {
        geometry_msgs::msg::Point world_point = camera_point;
        
        if (is_sim_)
        {
            // Apply simulation-specific transformations
            double pitch = 23.5 * M_PI / 180.0;
            double temp_y = 1.5 - camera_point.z * std::sin(pitch) - camera_point.y * std::cos(pitch);
            world_point.y = camera_point.z;
            world_point.z = temp_y;
        }
        else
        {
            // Use TF2 transformation for real robot
            try
            {
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header.stamp = depth_stamp_;
                point_in.header.frame_id = "base_link";
                point_in.point = camera_point;
                
                geometry_msgs::msg::TransformStamped transform_stamped = 
                    tf_buffer_->lookupTransform("odom", depth_frame_id_, depth_stamp_, tf2::durationFromSec(0.1));
                
                tf2::doTransform(point_in, point_out, transform_stamped);
                world_point = point_out.point;
            }
            catch (tf2::TransformException& e)
            {
                RCLCPP_ERROR(this->get_logger(), "Transform error: %s", e.what());
                return False
            }
        }
        
        // Apply robot position transformation (cloud_to_global equivalent)
        return cloudToGlobal(world_point);
    }

    geometry_msgs::msg::Point cloudToGlobal(const geometry_msgs::msg::Point& point)
    {
        geometry_msgs::msg::Point global_point;
        
        // Convert quaternion to yaw
        double bot_yaw = quaternionToYaw(bot_orientation_);
        
        // Transform point to global coordinates
        global_point.x = bot_position_.x + point.x * std::sin(bot_yaw) + point.y * std::cos(bot_yaw);
        global_point.y = bot_position_.y + point.y * std::sin(bot_yaw) - point.x * std::cos(bot_yaw);
        global_point.z = point.z;
        
        return global_point;
    }

    double quaternionToYaw(const geometry_msgs::msg::Quaternion& quat)
    {
        // Convert quaternion to yaw angle
        double siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y);
        double cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BarrelDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
