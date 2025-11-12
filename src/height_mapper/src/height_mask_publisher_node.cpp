#include <rclcpp/rclcpp.hpp>
#include "rclcpp/qos.hpp"
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"  

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
    std::string name;
    std::string topic_name;
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

        this->declare_parameter("depth_sub_topic", rclcpp::PARAMETER_STRING);
        std::string depth_sub_topic = this->get_parameter("depth_sub_topic").as_string();
    
        this->declare_parameter("color_sub_topic", rclcpp::PARAMETER_STRING);
        std::string color_sub_topic = this->get_parameter("color_sub_topic").as_string();

        this->declare_parameter("camera_info_sub_topic", rclcpp::PARAMETER_STRING);
        std::string camera_info_sub_topic = this->get_parameter("camera_info_sub_topic").as_string();
        
        // object_types.push_back(ObjectType("tyre", "tyre", 0.1f, 0.6f));
        object_types.push_back(ObjectType("traffic_drum", "traffic_drum", 0.5f, 1.2f));
        object_types.push_back(ObjectType("pedestrian", "pedestrian", 1.5f, 2.0f));
        
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            color_sub_topic, 10, 
            std::bind(&HeightMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
    
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            depth_sub_topic, 10, 
            std::bind(&HeightMaskPublisherNode::depthImageCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_sub_topic, 10, 
            std::bind(&HeightMaskPublisherNode::cameraInfoCallback, this, std::placeholders::_1));
    
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&HeightMaskPublisherNode::odomCallback, this, std::placeholders::_1));
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        rgb_recv = false;
        depth_recv = false;
        camera_info_recv = false;
        odom_recv = false;

        for (const auto& obj_type : object_types) {
            mask_publishers[obj_type.name] = this->create_publisher<sensor_msgs::msg::Image>(
                "/height_mask/" + obj_type.topic_name, 10);
            
            centroid_publishers[obj_type.name] = this->create_publisher<geometry_msgs::msg::Point>(
                "/detector/" + obj_type.topic_name + "/coordinates", 10);
            
            RCLCPP_INFO(this->get_logger(), "Created publisher for /detector/%s/coordinates", 
                        obj_type.topic_name.c_str());
        }
        
        stats_pub = this->create_publisher<std_msgs::msg::String>("/detector/stats", 10);
        
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mask/pointcloud", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&HeightMaskPublisherNode::timer_callback, this));
            
        log_odds_timer = this->create_wall_timer(
            std::chrono::milliseconds(150), std::bind(&HeightMaskPublisherNode::log_odds_callback, this));
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
    
    bool convert_depth_to_point(double u, double v, const double& depth, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info, Point odom_point) {
        double fx = camera_info->k[0]; 
        double fy = camera_info->k[4];
        double cx = camera_info->k[2];
        double cy = camera_info->k[5];
        double z = depth;
        double x = ((u-cx)*z)/fx;
        double y = ((v-cy)*z)/fy;

        geometry_msgs::msg::PointStamped point_in, point_out;
        point_in.header.frame_id = depth_image_msg->header.frame_id;
        point_in.header.stamp = depth_image_msg->header.stamp;
        point_in.point.x = x;
        point_in.point.y = y;
        point_in.point.z = z;
        
        try {
            // Transform to odom frame
            tf2::doTransform(point_in, point_out, transformStamped);
            odom_point.x = point_out.point.x;
            odom_point.y = point_out.point.y;
            odom_point.z = point_out.point.z;
            return true;
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
            return false;
        }
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
            throw std::runtime_error("Singular matrix in baseLinkToCloudPoint");
        }

        float rhs1 = base_link_point.x;
        float rhs2 = base_link_point.z - 1.5f;

        cloud_point.z = (rhs1 * D - B * rhs2) / det;
        cloud_point.y = (A * rhs2 - rhs1 * C) / det;

        return cloud_point;
    }

    Point cloudPointToGlobalPoint(Point &cloud_point, const BotPosition &bot_pose) {
        Point global_point;
        global_point.x = (bot_pose.x + cloud_point.x * sin(bot_pose.yaw) +
                        cloud_point.z * cos(bot_pose.yaw));
        global_point.y = (bot_pose.y + cloud_point.z * sin(bot_pose.yaw) -
                        cloud_point.x * cos(bot_pose.yaw));
        if(this->sim) {
            global_point.z = -1 * cloud_point.y + 1.5;    
        } else {
            global_point.z = - cloud_point.y;
        }
        return global_point;
    }

    double distance(Point a, Point b){
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    void computeObstacleLabels(const cv::Mat& depth_image, sensor_msgs::msg::CameraInfo::SharedPtr camera_info, cv::Mat& obstacle_labels, std::vector<float>& max_obstacle_heights, std::vector<float>& min_obstacle_heights) {

        obstacle_labels = cv::Mat::zeros(depth_image.rows, depth_image.cols, CV_32S);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_pc(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<std::vector<int>> pixel_to_point_map(depth_image.rows, std::vector<int>(depth_image.cols, -1));
        int point_index = 0;

        for (int u = 0; u < depth_image.rows; u++) {
            for (int v = 0; v < depth_image.cols; v++) {
                double depth = static_cast<double>(depth_image.at<float>(u, v));
                if (!std::isfinite(depth) || depth <= 0) {
                    continue;
                }
                Point base_point;
                if (!convert_depth_to_point(v, u, depth, camera_info, base_point)){
                    continue;
                }
                if (this->sim){  
                base_point = cloudPointToBaselink(base_point);
                }
                if (base_point.x > 1.0 && base_point.x < 6 && std::abs(base_point.y) < 10 && base_point.z > 0.1 && base_point.z < 3.0) {
                    pcl::PointXYZ pcl_p;
                    pcl_p.x = base_point.x;
                    pcl_p.y = base_point.y;
                    pcl_p.z = base_point.z;
                    pcl_pc->points.push_back(pcl_p);
                    
                    pixel_to_point_map[u][v] = point_index;
                    point_index++;
                }
            }
        }
        
        pcl_pc->width = pcl_pc->points.size();
        pcl_pc->height = 1;
        pcl_pc->is_dense = false;

        // if (pcl_pc->points.empty()) {
        //     // RCLCPP_WARN(this->get_logger(), "Pointcloud is empty after filtering");
        //     max_obstacle_heights.clear();
        //     return;
        // }
        
        // RCLCPP_INFO(this->get_logger(), "Point cloud has %zu points after filtering", pcl_pc->points.size());
        
        std::vector<pcl::PointIndices> cluster_indices = performClustering(pcl_pc, 0.1, 500, 100000);
        
        cv::Mat labels = assignLabelsToDepthImage(cluster_indices, pixel_to_point_map, depth_image.size());
        
        int num_labels = cluster_indices.size() + 1; 
        max_obstacle_heights.resize(num_labels, 0.0f);
        min_obstacle_heights.resize(num_labels, std::numeric_limits<float>::infinity());
        
        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                int label = labels.at<int>(i, j);

                if (label > 0) {
                    double depthvalue = static_cast<double>(depth_image.at<float>(i, j));
                    if (std::isfinite(depthvalue) && depthvalue > 0) {
                        Point base_point;
                        if (!convert_depth_to_point(j, i, depthvalue, camera_info, base_point)){
                            continue;
                        }
                        if (this->sim){
                        base_point = cloudPointToBaselink(base_point);
                        }
                        obstacle_labels.at<int>(i, j) = label;
                        
                        if (label < max_obstacle_heights.size()) {
                            min_obstacle_heights[label] = std::min(min_obstacle_heights[label], static_cast<float>(base_point.z));
                            max_obstacle_heights[label] = std::max(max_obstacle_heights[label], static_cast<float>(base_point.z));
                        }
                    }
                }
            }
        }
        
        // RCLCPP_INFO(this->get_logger(), "Found %zu clusters with heights:", cluster_indices.size());
        // for (size_t i = 1; i < max_obstacle_heights.size(); i++) {
        //     RCLCPP_INFO(this->get_logger(), "  Cluster %zu: height %.2fm", i, max_obstacle_heights[i]);
        // }
    }


    std::vector<pcl::PointIndices> performClustering(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double cluster_tolerance, int min_cluster_size, int max_cluster_size) {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_cluster_size);
        ec.setMaxClusterSize(max_cluster_size);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);
        
        return cluster_indices;
    }

    cv::Mat assignLabelsToDepthImage(const std::vector<pcl::PointIndices>& cluster_indices, const std::vector<std::vector<int>>& pixel_to_point_map, const cv::Size& image_size) {
        cv::Mat label_image = cv::Mat::zeros(image_size, CV_32S);
        
        int max_point_idx = 0;
        for (int i = 0; i < pixel_to_point_map.size(); i++) {
            for (int j = 0; j < pixel_to_point_map[i].size(); j++) {
                if (pixel_to_point_map[i][j] > max_point_idx) {
                    max_point_idx = pixel_to_point_map[i][j];
                }
            }
        }
        
        std::vector<int> point_to_cluster_label(max_point_idx + 1, 0);
        
        for (size_t cluster_id = 0; cluster_id < cluster_indices.size(); ++cluster_id) {
            for (const auto& point_idx : cluster_indices[cluster_id].indices) {
                if (point_idx < point_to_cluster_label.size()) {
                    point_to_cluster_label[point_idx] = cluster_id + 1;
                }
            }
        }
        
        for (int v = 0; v < image_size.height; ++v) {
            for (int u = 0; u < image_size.width; ++u) {
                int point_idx = pixel_to_point_map[v][u];
                if (point_idx >= 0 && point_idx < point_to_cluster_label.size()) {
                    label_image.at<int>(v, u) = point_to_cluster_label[point_idx];
                }
            }
        }
        
        return label_image; 
    }

    void publishBBOX(const std::string msg,rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher){
        std_msgs::msg::String message;
        message.data = msg;
        publisher->publish(message);
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
            
            // RCLCPP_INFO(this->get_logger(), "Processing images: RGB %dx%d, Depth %dx%d", 
                    //    rgb_image.cols, rgb_image.rows, depth_image.cols, depth_image.rows);
            
            std::lock_guard<std::mutex> lock(odom_mutex_);
            
            std::unordered_map<std::string, cv::Mat> masks;
            for (const auto& obj_type : object_types) {
                masks[obj_type.name] = cv::Mat(rgb_image.rows, rgb_image.cols, CV_8U, cv::Scalar(0));
            }
            
            cv::Mat obstacle_labels;
            std::vector<float> max_obstacle_heights;
            std::vector<float> min_obstacle_heights;
            
            computeObstacleLabels(depth_image, camera_info, obstacle_labels, max_obstacle_heights, min_obstacle_heights);
            
            std::vector<std::string> obstacle_types(max_obstacle_heights.size(), "");
            for (size_t i = 1; i < max_obstacle_heights.size(); i++) {
                float max_height = max_obstacle_heights[i];
                float min_height = min_obstacle_heights[i];
                RCLCPP_INFO(this->get_logger(), "MAX HEIGHT: %f", max_height);
                RCLCPP_INFO(this->get_logger(), "MIN_HEIGHT: %f", min_height);
                RCLCPP_INFO(this->get_logger(), "HEIGHT %d : %f",i, abs(max_height - min_height));
                for (const auto& obj_type : object_types) {
                    if (abs(max_height - min_height) > obj_type.min_height && abs(max_height - min_height) < obj_type.max_height) {
                        obstacle_types[i] = obj_type.name;
                        // RCLCPP_INFO(this->get_logger(), "Cluster %zu (height %.2fm) classified as %s", 
                        //            i, max_height, obj_type.name.c_str());
                        break;
                    }
                }
                
                if (obstacle_types[i].empty()) {
                    // RCLCPP_INFO(this->get_logger(), "Cluster %zu (height %.2fm) not classified", i, max_height);
                }
            }
            
            std::unordered_map<std::string, std::vector<Centroid3D>> type_centroids;
            std::unordered_map<std::string, std::unordered_map<int, int>> type_centroid_indices;
            
            for (const auto& obj_type : object_types) {
                type_centroids[obj_type.name] = std::vector<Centroid3D>();
            }
            
            for (size_t i = 1; i < obstacle_types.size(); i++) {
                const std::string& type = obstacle_types[i];
                if (!type.empty()) {
                    type_centroid_indices[type][i] = type_centroids[type].size();
                    type_centroids[type].push_back(Centroid3D());
                }
            }

            std::vector<std::vector<int>> ObstacleStats(obstacle_types.size(), std::vector<int>(4, -1));//xmin, xmax, ymin, ymax
            int mask_pixel_count = 0;
            for (int i = 0; i < depth_image.rows; i++) {
                for (int j = 0; j < depth_image.cols; j++) {
                    int label = obstacle_labels.at<int>(i, j);
                    if (label > 0 && label < obstacle_types.size()) { 
                        double depthvalue = static_cast<double>(depth_image.at<float>(i, j));
                        if (std::isfinite(depthvalue) && depthvalue > 0) {
                            const std::string& type = obstacle_types[label];
                            
                            if (!type.empty()) {
                                auto& stats = ObstacleStats[label];
                                
                                if (stats[0] == -1 || j < stats[0]) stats[0] = j;
                                if (j > stats[1]) stats[1] = j;
                                if (stats[2] == -1 || i < stats[2]) stats[2] = i;
                                if (i > stats[3]) stats[3] = i;
                                Point base_point;
                                if (!convert_depth_to_point(j, i, depthvalue, camera_info, base_point)){
                                    continue;
                                }
                                if (this->sim){
                                    base_point = cloudPointToBaselink(base_point);
                                }
                                masks[type].at<uchar>(i, j) = 255;
                                mask_pixel_count++;
                                
                                if (type_centroid_indices[type].find(label) != type_centroid_indices[type].end()) {
                                    int centroid_idx = type_centroid_indices[type][label];
                                    if (centroid_idx < type_centroids[type].size()) {
                                        type_centroids[type][centroid_idx].addPoint(
                                            base_point.x, base_point.y, base_point.z);
                                    }
                                }
                            }
                        }
                    }
                }
            }

            std::ostringstream stats_summary;
            for (int i = 0; i < ObstacleStats.size(); i++){
                if (ObstacleStats[i][0] != -1){
                    RCLCPP_INFO(this->get_logger(), "OBSTACLE TYPE: %s %d %d %d %d", obstacle_types[i].c_str(),ObstacleStats[i][0], ObstacleStats[i][1], ObstacleStats[i][2], ObstacleStats[i][3]);
                    int x = ObstacleStats[i][0];
                    int y = ObstacleStats[i][2];
                    int width = ObstacleStats[i][1] - ObstacleStats[i][0];
                    int height = ObstacleStats[i][3] - ObstacleStats[i][2];

                    stats_summary << "Label: " << i << ", Type: " << obstacle_types[i] << ", X: " << x << ", Y: " << y << ", Width: " << width << ", Height: " << height << "\n";
                }
            }

        std::string msg_str = stats_summary.str();
        if(!msg_str.empty() && msg_str.back() == '\n')msg_str.pop_back();
        if (!msg_str.empty())publishBBOX(msg_str, stats_pub);
        //         }
        //     }
            // RCLCPP_INFO(this->get_logger(), "Total mask pixels set: %d", mask_pixel_count);
            
            rclcpp::Time current_time = this->now();
            
            for (const auto& obj_type : object_types) {
                int type_pixels = cv::countNonZero(masks[obj_type.name]);
                // RCLCPP_INFO(this->get_logger(), "Mask for %s has %d pixels", obj_type.name.c_str(), type_pixels);
                
                for (auto& centroid : type_centroids[obj_type.name]) {
                    centroid.finalize();
                    
                    if (centroid.point_count > 100) {
                        Point base_point = {centroid.x, centroid.y, centroid.z};
                        if (this->sim){
                            base_point = baseLinkToCloudPoint(base_point);
                        }
                        Point global_point = cloudPointToGlobalPoint(base_point, bot_pose);

                        bool matched = false;
                        for (auto& detection : object_detections) {
                            if (detection.type == obj_type.name && distance(detection.global_position, global_point) < 1.0) {
                                detection.log_odds += 0.3;
                                if (detection.log_odds > 5.0) {
                                    detection.log_odds = 5.0;
                                }
                                RCLCPP_INFO(this->get_logger(), "Updated %s detection at (%.2f, %.2f, %.2f), log odds now: %.2f",
                                        detection.type.c_str(), global_point.x, global_point.y, global_point.z, detection.log_odds);

                                detection.last_update = current_time;
                                detection.global_position = global_point;
                                matched = true;
                                break;
                            }
                        }
                        
                        if (!matched) {
                            double initial_log_odds = 0.0;
                            ObjectDetection new_detection(obj_type.name, global_point, initial_log_odds, current_time);
                            object_detections.push_back(new_detection);
                            RCLCPP_INFO(this->get_logger(), "New %s detection at (%.2f, %.2f, %.2f)", 
                                      obj_type.name.c_str(), global_point.x, global_point.y, global_point.z);
                        }
                    }
                }
                
                for (const auto& detection : object_detections) {
                    if (detection.type == obj_type.name && detection.log_odds > 4.0) {
                        publishCentroids(detection.global_position, centroid_publishers[obj_type.name]);
                    }
                }
                
                auto mask_msg = cv_bridge::CvImage(depth->header, "mono8", masks[obj_type.name]).toImageMsg();
                mask_publishers[obj_type.name]->publish(*mask_msg);
            }
            
            // RCLCPP_INFO(this->get_logger(), "Processing time: %s", t.log().c_str());
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in publish_mask: %s", e.what());
        }
    }

    void publishCentroids(Point centroid, 
                         rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher) {
                geometry_msgs::msg::Point point_msg;
                Point centroid_point = {centroid.x, centroid.y, centroid.z};
                point_msg.x = centroid_point.x;
                point_msg.y = centroid_point.y;
                point_msg.z = centroid_point.z;
                publisher->publish(point_msg);
                RCLCPP_DEBUG(this->get_logger(), "Published centroid at (%.2f, %.2f, %.2f)",
                           centroid.x, centroid.y, centroid.z);
    }
    void publish_pointcloud(const sensor_msgs::msg::Image::ConstSharedPtr depth,
                            const sensor_msgs::msg::CameraInfo::SharedPtr camera_info,
                            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub) 
    {
        
        cv_bridge::CvImagePtr depth_image_ptr;
        try {
            depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV bridge exception in point cloud publisher: %s", e.what());
            return;
        }
        
        cv::Mat depth_image = depth_image_ptr->image;
        int width = depth->width;
        int height = depth->height;

        
        pcl::PointCloud<pcl::PointXYZ> cloud;
        cloud.width = width;
        cloud.height = height;
        cloud.is_dense = false;
        cloud.points.resize(width * height);

        
        float minx, miny, minz;
        float maxx, maxy, maxz;
        bool first = true;

        for (int v = 0; v < height; ++v) {
            for (int u = 0; u < width; ++u) {
                float depth_value = depth_image.at<float>(v, u);  
                
                pcl::PointXYZ& pt = cloud.at(u, v);

                if (!std::isfinite(depth_value) || depth_value <= 0) {
                    pt.x = pt.y = pt.z = std::numeric_limits<float>::quiet_NaN();
                    continue;
                }

                Point p;
                if (!convert_depth_to_point(u, v, depth_value, camera_info, p)){
                    continue;
                }
                
                pt.x = static_cast<float>(p.x);
                pt.y = static_cast<float>(p.y);
                pt.z = static_cast<float>(p.z);
                if (first) {
                    minx = static_cast<float>(p.x);
                    miny = static_cast<float>(p.y);
                    minz = static_cast<float>(p.z);
                    maxx = static_cast<float>(p.x);
                    maxy = static_cast<float>(p.y);
                    maxz = static_cast<float>(p.z);
                    first = false;
                } else{
                    if(pt.x > maxx) {
                        maxx = pt.x;
                    }
                    if(pt.y > maxy) {
                        maxy = pt.y;
                    }
                    if(pt.z > maxz) {
                        maxz = pt.z;
                    }
                    if(pt.x < minx) {
                        minx = pt.x;
                    }
                    if(pt.y < miny) {
                        miny = pt.y;
                    }
                    if(pt.z < minz) {
                        minz = pt.z;
                    }
                }
            }
        }

        
        sensor_msgs::msg::PointCloud2 msg;
        pcl::toROSMsg(cloud, msg);
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";  
        
        RCLCPP_INFO(this->get_logger(), "x: %f to %f", minx, maxx);
        RCLCPP_INFO(this->get_logger(), "y: %f to %f", miny, maxy);
        RCLCPP_INFO(this->get_logger(), "z: %f to %f", minz, maxz);

        
        pub->publish(msg);
        RCLCPP_DEBUG(this->get_logger(), "Published PointCloud2 with %d points", width * height);
    }

    void timer_callback() {
        
        bool recv = rgb_recv && depth_recv && camera_info_recv; 
        if (recv) {
            try {
                try{
                    transformStamped = tf_buffer_->lookupTransform("odom", depth_image_msg->header.frame_id, depth_image_msg->header.stamp);
                } catch (tf2::TransformException& ex){
                    transformStamped = tf_buffer_ ->lookupTransform("odom", depth_image_msg->header.frame_id, tf2::TimePointZero);
                }
                publish_mask(rgb_image_msg, depth_image_msg, camera_info_msg);
                publish_pointcloud(depth_image_msg, camera_info_msg, pointcloud_pub);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in timer_callback: %s", e.what());
            }
        } else {
            
            if (!rgb_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for RGB image");
            if (!depth_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for depth image");
            if (!camera_info_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for camera info");
            
        }
    }

    void log_odds_callback() {
        rclcpp::Time current_time = this->now();
        std::vector<ObjectDetection> updated_detections;

        for (auto& detection : object_detections) {   

            detection.log_odds -= 0.1;
            if (detection.log_odds > -1.0) {
                updated_detections.push_back(detection);
            } else {
                RCLCPP_INFO(this->get_logger(), "Removed %s detection at (%.2f, %.2f, %.2f), log odds: %.2f", detection.type.c_str(), detection.global_position.x, detection.global_position.y, detection.global_position.z, detection.log_odds);
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr stats_pub;
    
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg, depth_image_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr log_odds_timer;  

    cv_bridge::CvImagePtr rgb_image_ptr, depth_image_ptr;
    cv::Mat rgb_image, depth_image;
    
    geometry_msgs::msg::TransformStamped transformStamped;


    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    bool sim;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
