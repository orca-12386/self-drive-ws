#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <mutex>
#include <tf2/LinearMath/Quaternion.h>  // Added for quaternion handling
#include <tf2/LinearMath/Matrix3x3.h>

#ifdef DEBUG
#define DEBUG_LOG(logger, msg) RCLCPP_INFO(logger, msg)
#else
#define DEBUG_LOG(logger, msg)
#endif


struct BotPosition {
    double x, y, z;
    double yaw;
};

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

class Centroid3D {
public:
    Centroid3D() : x(0.0), y(0.0), z(0.0), point_count(0) {}
    
    void addPoint(double px, double py, double pz) {
        x_sum += px;
        y_sum += py;
        z_sum += pz;
        point_count++;
    }
    
    void finalize() {
        if (point_count > 0) {
            x = x_sum / point_count;
            y = y_sum / point_count;
            z = z_sum / point_count;
        }
    }
    
    double x, y, z;
    int point_count;
    
private:
    double x_sum = 0.0;
    double y_sum = 0.0;
    double z_sum = 0.0;
};

struct ObjectType {
    std::string name;         // Internal name
    std::string topic_name;   // Name used in ROS topics
    float min_height;
    float max_height;
    
    ObjectType(const std::string& name, const std::string& topic_name, float min_height, float max_height)
        : name(name), topic_name(topic_name), min_height(min_height), max_height(max_height) {}
};

typedef struct Point {
    double x;
    double y;
    double z;
} Point;

struct ObjectDetection {
    std::string type;
    Point global_position;
    double log_odds;
    rclcpp::Time last_update;
    
    ObjectDetection(const std::string& type, const Point& pos, double initial_log_odds, const rclcpp::Time& time)
        : type(type), global_position(pos), log_odds(initial_log_odds), last_update(time) {}
};

// Modified HeightMaskPublisherNode class with fixes

class HeightMaskPublisherNode : public rclcpp::Node
{
public:
    HeightMaskPublisherNode() : 
    rclcpp::Node("height_mask_publisher_node"),
    pitch(23.5f * M_PI / 180.0f),
    cos_pitch(cos(pitch)),
    sin_pitch(sin(pitch))
    {
    
        RCLCPP_INFO(this->get_logger(), "height_mask_publisher_node started");
        this->declare_parameter("sim", rclcpp::PARAMETER_BOOL);
        sim = this->get_parameter("sim").as_bool();
        // Define object types with their height ranges
        object_types.push_back(ObjectType("tyre", "tyre", 0.1f, 0.6f));
        object_types.push_back(ObjectType("traffic_drum", "traffic_drum", 0.6f, 1.2f));
        // object_types.push_back(ObjectType("stop_sign", "stop_sign", 1.4f, 1.8f));
        object_types.push_back(ObjectType("stop_sign", "stop_sign", 1.8f, 2.2f));
        
        // Create subscriptions
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/image_raw", 10, 
            std::bind(&HeightMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
    
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/depth/image_raw", 10, 
            std::bind(&HeightMaskPublisherNode::depthImageCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed_node/stereocamera/camera_info", 10, 
            std::bind(&HeightMaskPublisherNode::cameraInfoCallback, this, std::placeholders::_1));
    
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&HeightMaskPublisherNode::odomCallback, this, std::placeholders::_1));
        
        // Initialize flags
        rgb_recv = false;
        depth_recv = false;
        camera_info_recv = false;
        odom_recv = false;

        // Create publishers for each object type
        for (const auto& obj_type : object_types) {
            // Create mask image publisher
            mask_publishers[obj_type.name] = this->create_publisher<sensor_msgs::msg::Image>(
                "/height_mask/" + obj_type.topic_name, 10);
            
            // Create centroid publisher
            centroid_publishers[obj_type.name] = this->create_publisher<geometry_msgs::msg::Point>(
                "/detector/" + obj_type.topic_name + "/coordinates", 10);
            
            // Debug log creation of publishers
            RCLCPP_INFO(this->get_logger(), "Created publisher for /detector/%s/coordinates", 
                        obj_type.topic_name.c_str());
        }
        
        // Create point cloud publisher
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mask/pointcloud", 10);

        // Create timers
        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&HeightMaskPublisherNode::timer_callback, this));
            
        // Add a timer for log odds updating
        log_odds_timer = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&HeightMaskPublisherNode::log_odds_callback, this));
    };

private:
    const float pitch;
    const float cos_pitch;
    const float sin_pitch;
    
    std::vector<ObjectType> object_types;
    std::unordered_map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> mask_publishers;
    std::unordered_map<std::string, rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr> centroid_publishers;
    std::unordered_map<std::string, sensor_msgs::msg::Image::SharedPtr> mask_messages;
    BotPosition bot_pose;
    std::mutex odom_mutex_; 
    std::vector<ObjectDetection> object_detections;
    
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->rgb_image_msg = msg;
        rgb_recv = true;
        RCLCPP_DEBUG(this->get_logger(), "Received RGB image");
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->depth_image_msg = msg;
        depth_recv = true;
        RCLCPP_DEBUG(this->get_logger(), "Received depth image");
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->camera_info_msg = msg;
        camera_info_recv = true;
        RCLCPP_DEBUG(this->get_logger(), "Received camera info");
    }
    
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        bot_pose.x = msg->pose.pose.position.x;
        bot_pose.y = msg->pose.pose.position.y;
        bot_pose.z = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, 
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, 
            msg->pose.pose.orientation.w);
        double roll, pitch;
        tf2::Matrix3x3(q).getRPY(roll, pitch, bot_pose.yaw);
        odom_recv = true;
        RCLCPP_DEBUG(this->get_logger(), "Received odometry");
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    static double logodds_to_prob(double log_odds) {
        return 1.0 - (1.0 / (1.0 + std::exp(log_odds)));
    }

    static double prob_to_logodds(double probability) {
        return std::log(probability / (1.0 - probability));
    }
    
    Point convert_depth_to_point(double u, double v, const double& depth, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        double fx = camera_info->k[0]; 
        double fy = camera_info->k[4];
        double cx = camera_info->k[2];
        double cy = camera_info->k[5];
        double z = depth;
        double x = ((u-cx)*z)/fx;
        double y = ((v-cy)*z)/fy;
        Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
    
    Point cloudPointToGlobalPoint(Point &cloud_point, const BotPosition &bot_pose) {
        Point global_point;
        global_point.x = (bot_pose.x + cloud_point.x * sin(bot_pose.yaw) +
                        cloud_point.z * cos(bot_pose.yaw));
        global_point.y = (bot_pose.y + cloud_point.z * sin(bot_pose.yaw) -
                        cloud_point.x * cos(bot_pose.yaw));
        global_point.z = -1 * cloud_point.y + 1.5;
        return global_point;
    }

    Point cloudPointToBaselink(Point &cloud_point) {
        Point base_link_point;
        base_link_point.x = cloud_point.z * cos_pitch - cloud_point.y * sin_pitch;
        base_link_point.y = -cloud_point.x;
        base_link_point.z = 1.5f - cloud_point.z * sin_pitch - cloud_point.y * cos_pitch;
        return base_link_point;
    }

    Point baseLinkToCloudPoint(Point &base_link_point) {
        Point cloud_point;
        
        cloud_point.x = -base_link_point.y;

        float A = cos_pitch;
        float B = -sin_pitch;
        float C = -sin_pitch;
        float D = -cos_pitch;

        float det = A * D - B * C;

        if (std::abs(det) < 1e-6) {
            // Handle singularity or throw error
            throw std::runtime_error("Singular matrix in baseLinkToCloudPoint");
        }

        float rhs1 = base_link_point.x;
        float rhs2 = base_link_point.z - 1.5f;

        cloud_point.z = (rhs1 * D - B * rhs2) / det;
        cloud_point.y = (A * rhs2 - rhs1 * C) / det;

        return cloud_point;
    }

    double distance(Point a, Point b){
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    void computeObstacleLabels(const cv::Mat& depth_image, sensor_msgs::msg::CameraInfo::SharedPtr camera_info, 
                              cv::Mat& obstacle_labels, std::vector<float>& max_obstacle_heights, cv::Mat& centroids) {

        obstacle_labels = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_32S);
        
        cv::Mat valid_points = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_8U);
        
        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                double depthvalue = static_cast<double>(depth_image.at<float>(i, j));
                
                if (!std::isfinite(depthvalue) || depthvalue <= 0) {
                    continue;
                }
                
                Point base_point = convert_depth_to_point(j, i, depthvalue, camera_info);
                if (this->sim){
                base_point = cloudPointToBaselink(base_point);
                }
                if (base_point.x > 1.5 && base_point.x < 15 && base_point.z > 0.1 && base_point.z < 2.0) {
                    valid_points.at<uchar>(i, j) = 255;
                }
            }
        }
        
        cv::Mat labels, stats, centroids_mat;
        int num_labels = cv::connectedComponentsWithStats(valid_points, labels, stats, centroids_mat, 8); // basically clustering
        
        centroids = centroids_mat; // Store centroids for later use
        max_obstacle_heights.resize(num_labels, 0.0f);
        
        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                int label = labels.at<int>(i, j);

                if (label > 0) {
                    double depthvalue = static_cast<double>(depth_image.at<float>(i, j));
                    if (std::isfinite(depthvalue) && depthvalue > 0) {
                        Point base_point = convert_depth_to_point(j, i, depthvalue, camera_info);
                        if (this->sim){
                        base_point = cloudPointToBaselink(base_point);
                        }
                        obstacle_labels.at<int>(i, j) = label;
                        
                        max_obstacle_heights[label] = std::max(max_obstacle_heights[label], static_cast<float>(base_point.z));
                    }
                }
            }
        }
        
        RCLCPP_DEBUG(this->get_logger(), "Found %d connected components", num_labels);
    }

    void publish_mask(sensor_msgs::msg::Image::SharedPtr rgb, sensor_msgs::msg::Image::SharedPtr depth, 
                     sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {    
        Timer t = Timer("sensor msg to cv mat");
        
        try {
            rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
            rgb_image = rgb_image_ptr->image;

            depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
            depth_image = depth_image_ptr->image;

            if (rgb_image.empty() || depth_image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty image received");
                return;
            }
            
            // Lock to safely access bot_pose during processing
            std::lock_guard<std::mutex> lock(odom_mutex_);
            
            // Create mask images for each object type
            std::unordered_map<std::string, cv::Mat> masks;
            for (const auto& obj_type : object_types) {
                masks[obj_type.name] = cv::Mat(rgb_image.rows, rgb_image.cols, CV_8U, cv::Scalar(0));
            }
            
            cv::Mat obstacle_labels;
            std::vector<float> max_obstacle_heights;
            cv::Mat centroids_mat;
            computeObstacleLabels(depth_image, camera_info, obstacle_labels, max_obstacle_heights, centroids_mat);
            
            // Classify obstacles based on height
            std::vector<std::string> obstacle_types(max_obstacle_heights.size(), "");
            for (size_t i = 1; i < max_obstacle_heights.size(); i++) {
                float max_height = max_obstacle_heights[i];
                
                for (const auto& obj_type : object_types) {
                    if (max_height >= obj_type.min_height && max_height <= obj_type.max_height) {
                        obstacle_types[i] = obj_type.name;
                        break;
                    }
                }
            }
            
            // Create centroids for each object type
            std::unordered_map<std::string, std::vector<Centroid3D>> type_centroids;
            std::unordered_map<std::string, std::unordered_map<int, int>> type_centroid_indices;
            
            for (const auto& obj_type : object_types) {
                type_centroids[obj_type.name] = std::vector<Centroid3D>();
            }
            
            // Initialize centroid data structures
            for (size_t i = 1; i < obstacle_types.size(); i++) {
                const std::string& type = obstacle_types[i];
                if (!type.empty()) {
                    type_centroid_indices[type][i] = type_centroids[type].size();
                    type_centroids[type].push_back(Centroid3D());
                }
            }

            // Process each point
            for (int i = 0; i < depth_image.rows; i++) {
                for (int j = 0; j < depth_image.cols; j++) {
                    int label = obstacle_labels.at<int>(i, j);
                    if (label > 0) { // Skip background
                        double depthvalue = static_cast<double>(depth_image.at<float>(i, j));
                        if (std::isfinite(depthvalue) && depthvalue > 0) {
                            const std::string& type = obstacle_types[label];
                            
                            if (!type.empty()) {
                                Point base_point = convert_depth_to_point(j, i, depthvalue, camera_info);
                                if (this->sim){
                                Point base_point = cloudPointToBaselink(base_point);
                                }
                                // Update mask
                                masks[type].at<uchar>(i, j) = 255;
                                
                                // Update centroid
                                type_centroids[type][type_centroid_indices[type][label]].addPoint(
                                    base_point.x, base_point.y, base_point.z);
                            }
                        }
                    }
                }
            }
            
            rclcpp::Time current_time = this->now();
            
            // Finalize centroids and publish masks and centroids
            for (const auto& obj_type : object_types) {
                std::vector<Centroid3D> confident_centroids;
                
                // First pass: update object detections
                for (auto& centroid : type_centroids[obj_type.name]) {
                    centroid.finalize();
                    
                    if (centroid.point_count > 100) { // Added threshold for point count
                        Point base_point = {centroid.x, centroid.y, centroid.z};
                        if (this->sim){
                        Point base_point = baseLinkToCloudPoint(base_point);
                        }
                        Point global_point = cloudPointToGlobalPoint(base_point, bot_pose);

                        bool matched = false;
                        for (auto& detection : object_detections) {
                            if (detection.type == obj_type.name && distance(detection.global_position, global_point) < 1.0) {
                                detection.log_odds += prob_to_logodds(0.8);
                                if (detection.log_odds > 5.0) {
                                    detection.log_odds = 5.0;
                                }

                                detection.last_update = current_time;
                                detection.global_position = global_point; // Update position
                                matched = true;
                                break;
                            }
                        }
                        
                        if (!matched) {
                            // Start with higher initial probability for faster convergence
                            double initial_log_odds = prob_to_logodds(0.5);
                            ObjectDetection new_detection(obj_type.name, global_point, initial_log_odds, current_time);
                            object_detections.push_back(new_detection);
                            RCLCPP_INFO(this->get_logger(), "New %s detection at (%.2f, %.2f, %.2f)", 
                                      obj_type.name.c_str(), global_point.x, global_point.y, global_point.z);
                        }
                    }
                }
                
                // Second pass: collect confident centroids
                for (const auto& detection : object_detections) {
                    if (detection.type == obj_type.name && logodds_to_prob(detection.log_odds) > 0.9) { // Lower threshold
                        // Convert back to camera frame for publishing
                        Point cloud_point;
                        Point base_point;
                        
                        // Find the closest centroid to this detection
                        for (auto& centroid : type_centroids[obj_type.name]) {
                            if (centroid.point_count > 100) {
                                confident_centroids.push_back(centroid);
                            }
                        }
                    }
                }
                
                // Publish centroids directly from confident_centroids
                if (!confident_centroids.empty()) {
                    RCLCPP_INFO(this->get_logger(), "Publishing %zu confident %s centroid(s)", 
                              confident_centroids.size(), obj_type.name.c_str());
                    publishCentroids(confident_centroids, centroid_publishers[obj_type.name]);
                }
                
                // Publish mask
                auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", masks[obj_type.name]).toImageMsg();
                mask_publishers[obj_type.name]->publish(*mask_msg);
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Processing time: %s", t.log().c_str());
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in publish_mask: %s", e.what());
        }
    }
    
    void publishCentroids(const std::vector<Centroid3D>& centroids, 
                         rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher) {
        for (const auto& centroid : centroids) {
            if (centroid.point_count > 0) {
                geometry_msgs::msg::Point point_msg;
                Point centroid_point = {centroid.x, centroid.y, centroid.z};
                if (this->sim){
                    centroid_point = baseLinkToCloudPoint(centroid_point); 
                }
                Point centroid_global = cloudPointToGlobalPoint(centroid_point, bot_pose);
                point_msg.x = centroid_global.x;
                point_msg.y = centroid_global.y;
                point_msg.z = centroid_global.z;
                publisher->publish(point_msg);
                RCLCPP_DEBUG(this->get_logger(), "Published centroid at (%.2f, %.2f, %.2f) with %d points",
                           centroid.x, centroid.y, centroid.z, centroid.point_count);
            }
        }
    }

    void timer_callback() {
        // Check if all required data is available
        bool recv = rgb_recv && depth_recv && camera_info_recv && odom_recv; // Added odometry check
        if (recv) {
            try {
                publish_mask(rgb_image_msg, depth_image_msg, camera_info_msg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in timer_callback: %s", e.what());
            }
        } else {
            // Log what's missing to help troubleshoot
            if (!rgb_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for RGB image");
            if (!depth_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for depth image");
            if (!camera_info_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for camera info");
            if (!odom_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for odometry");
        }
    }

    void log_odds_callback() {
        rclcpp::Time current_time = this->now();
        std::vector<ObjectDetection> updated_detections;

        for (auto& detection : object_detections) {
            if (current_time - detection.last_update > rclcpp::Duration::from_seconds(1.0)) {
                detection.log_odds -= prob_to_logodds(0.3);
            }

            if (detection.log_odds > -1.0) {
                updated_detections.push_back(detection);
            } else {
                RCLCPP_INFO(this->get_logger(), "Removed %s detection", detection.type.c_str());
            }
        }
        object_detections = updated_detections;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub, depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    bool rgb_recv;
    bool depth_recv;
    bool camera_info_recv;
    bool odom_recv;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg, depth_image_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr log_odds_timer;  // Added timer for log odds

    cv_bridge::CvImagePtr rgb_image_ptr, depth_image_ptr;
    cv::Mat rgb_image, depth_image;
    bool sim;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}