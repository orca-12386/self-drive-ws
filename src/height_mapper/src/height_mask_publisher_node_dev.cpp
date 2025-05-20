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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>

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
    pitch(18.0f * M_PI / 180.0f),
    cos_pitch(cos(pitch)),
    sin_pitch(sin(pitch))
    {
        RCLCPP_INFO(this->get_logger(), "height_mask_publisher_node started");
        this->declare_parameter("sim", rclcpp::PARAMETER_BOOL);
        sim = this->get_parameter("sim").as_bool();
        
        object_types.push_back(ObjectType("tyre", "tyre", 0.1f, 0.6f));
        object_types.push_back(ObjectType("traffic_drum", "traffic_drum", 0.6f, 1.2f));
        object_types.push_back(ObjectType("stop_sign", "stop_sign", 1.8f, 2.2f));
        
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/rgb/image_rect_color", 10, 
            std::bind(&HeightMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));

        // Subscribe to the point cloud topic directly
        pointcloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/zed_node/point_cloud/cloud_registered", 10, 
            std::bind(&HeightMaskPublisherNode::pointCloudCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed_node/rgb/camera_info", 10, 
            std::bind(&HeightMaskPublisherNode::cameraInfoCallback, this, std::placeholders::_1));
    
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&HeightMaskPublisherNode::odomCallback, this, std::placeholders::_1));

        rgb_recv = false;
        pointcloud_recv = false;
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

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        this->pointcloud_msg = msg;
        pointcloud_recv = true;
        RCLCPP_DEBUG(this->get_logger(), "Received point cloud");
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
    
    Point cloudPointToGlobalPoint(Point &cloud_point, const BotPosition &bot_pose) {
        Point global_point;
        global_point.x = (bot_pose.x + cloud_point.x * sin(bot_pose.yaw) +
                        cloud_point.z * cos(bot_pose.yaw));
        global_point.y = (bot_pose.y + cloud_point.z * sin(bot_pose.yaw) -
                        cloud_point.x * cos(bot_pose.yaw));
        if(this->sim) {
            global_point.z = -1 * cloud_point.y + 1.5;    
        } else {
            global_point.z = cloud_point.y;
        }
        return global_point;
    }

    Point cloudPointToBaselink(Point &cloud_point) {
        Point base_link_point;
        base_link_point.x = cloud_point.z - cloud_point.y;
        base_link_point.y = -cloud_point.x;
        if(this->sim) {
            base_link_point.z = 1.5f - cloud_point.z * sin_pitch - cloud_point.y * cos_pitch;        
        } else {
            base_link_point.z = 1.5f - cloud_point.z * sin_pitch - cloud_point.y * cos_pitch;        
        }
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

    double distance(Point a, Point b){
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    
    void process_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg, 
                           const sensor_msgs::msg::Image::SharedPtr rgb_msg) {
        Timer t = Timer("process_pointcloud");
        
        try {
            // Convert RGB image to OpenCV format for mask creation
            rgb_image_ptr = cv_bridge::toCvCopy(rgb_msg, rgb_msg->encoding);
            rgb_image = rgb_image_ptr->image;
            
            if (rgb_image.empty()) {
                RCLCPP_WARN(this->get_logger(), "Empty RGB image received");
                return;
            }
            
            // Create masks for each object type
            std::unordered_map<std::string, cv::Mat> masks;
            for (const auto& obj_type : object_types) {
                masks[obj_type.name] = cv::Mat(rgb_image.rows, rgb_image.cols, CV_8U, cv::Scalar(0));
            }
            
            // Convert ROS PointCloud2 to PCL PointCloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*cloud_msg, *pcl_cloud);
            
            // Apply voxel grid filter to downsample the point cloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
            voxel_filter.setInputCloud(pcl_cloud);
            voxel_filter.setLeafSize(0.01f, 0.01f, 0.01f);  // 1cm voxel size
            voxel_filter.filter(*cloud_filtered);
            
            // Euclidean Cluster Extraction
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud(cloud_filtered);
            
            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance(0.05);  // 5cm
            ec.setMinClusterSize(100);
            ec.setMaxClusterSize(25000);
            ec.setSearchMethod(tree);
            ec.setInputCloud(cloud_filtered);
            ec.extract(cluster_indices);
            
            // Process each cluster
            std::unordered_map<std::string, std::vector<Centroid3D>> type_centroids;
            for (const auto& obj_type : object_types) {
                type_centroids[obj_type.name] = std::vector<Centroid3D>();
            }
            
            std::lock_guard<std::mutex> lock(odom_mutex_);
            
            for (const auto& indices : cluster_indices) {
                // Calculate cluster height
                float min_z = std::numeric_limits<float>::max();
                float max_z = -std::numeric_limits<float>::max();
                
                Centroid3D centroid;
                
                for (const auto& idx : indices.indices) {
                    const auto& point = cloud_filtered->points[idx];
                    
                    // Convert to baselink coordinates for height measurement
                    Point cloud_point = {point.x, point.y, point.z};
                    Point base_point = cloudPointToBaselink(cloud_point);
                    
                    min_z = std::min(min_z, static_cast<float>(base_point.z));
                    max_z = std::max(max_z, static_cast<float>(base_point.z));
                    
                    centroid.addPoint(base_point.x, base_point.y, base_point.z);
                }
                
                centroid.finalize();
                float cluster_height = max_z - min_z;
                
                // Determine object type based on height
                std::string detected_type = "";
                for (const auto& obj_type : object_types) {
                    if (max_z >= obj_type.min_height && max_z <= obj_type.max_height) {
                        detected_type = obj_type.name;
                        break;
                    }
                }
                
                if (!detected_type.empty() && centroid.point_count > 100) {
                    type_centroids[detected_type].push_back(centroid);
                    
                    // Project points to image for mask creation
                    for (const auto& idx : indices.indices) {
                        const auto& point = cloud_filtered->points[idx];
                        
                        // Project 3D point to 2D image coordinates using camera info
                        double fx = camera_info_msg->k[0];
                        double fy = camera_info_msg->k[4];
                        double cx = camera_info_msg->k[2];
                        double cy = camera_info_msg->k[5];
                        
                        // Simple pinhole projection
                        int u = static_cast<int>((fx * point.x / point.z) + cx);
                        int v = static_cast<int>((fy * point.y / point.z) + cy);
                        
                        // Check if point projects within image bounds
                        if (u >= 0 && u < rgb_image.cols && v >= 0 && v < rgb_image.rows) {
                            masks[detected_type].at<uchar>(v, u) = 255;
                        }
                    }
                }
            }
            
            // Process centroids and update object detections
            rclcpp::Time current_time = this->now();
            
            for (const auto& obj_type : object_types) {
                for (auto& centroid : type_centroids[obj_type.name]) {
                    if (centroid.point_count > 100) {
                        Point base_point = {centroid.x, centroid.y, centroid.z};
                        Point cloud_point = baseLinkToCloudPoint(base_point);
                        Point global_point = cloudPointToGlobalPoint(cloud_point, bot_pose);
                        
                        bool matched = false;
                        for (auto& detection : object_detections) {
                            if (detection.type == obj_type.name && distance(detection.global_position, global_point) < 0.8) {
                                detection.log_odds += 0.1;
                                if (detection.log_odds > 5.0) {
                                    detection.log_odds = 5.0;
                                }
                                
                                RCLCPP_DEBUG(this->get_logger(), "Updated %s detection at (%.2f, %.2f, %.2f), log odds now: %.2f",
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
                
                // Publish confident detections
                for (const auto& detection : object_detections) {
                    if (detection.type == obj_type.name && detection.log_odds > 4.0) {
                        publishCentroids(detection.global_position, centroid_publishers[obj_type.name]);
                    }
                }
                
                // Log detection status
                for (const auto& detection : object_detections) {
                    RCLCPP_INFO(this->get_logger(), "%s detection at (%.2f, %.2f, %.2f) has log odds: %.2f (probability: %.2f)", 
                                detection.type.c_str(), detection.global_position.x, detection.global_position.y, 
                                detection.global_position.z, detection.log_odds, logodds_to_prob(detection.log_odds));
                }
                
                // Publish masks
                auto mask_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", masks[obj_type.name]).toImageMsg();
                mask_publishers[obj_type.name]->publish(*mask_msg);
            }
            
            // Republish the processed point cloud
            pointcloud_pub->publish(*cloud_msg);
            
            RCLCPP_DEBUG(this->get_logger(), "Processing time: %s", t.log().c_str());
            
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in process_pointcloud: %s", e.what());
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

    void timer_callback() {
        bool recv = rgb_recv && pointcloud_recv && camera_info_recv && odom_recv; 
        if (recv) {
            try {
                process_pointcloud(pointcloud_msg, rgb_image_msg);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "Exception in timer_callback: %s", e.what());
            }
        } else {
            if (!rgb_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for RGB image");
            if (!pointcloud_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for point cloud");
            if (!camera_info_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for camera info");
            if (!odom_recv) RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for odometry");
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
                RCLCPP_INFO(this->get_logger(), "Removed %s detection at (%.2f, %.2f, %.2f), log odds: %.2f",
                        detection.type.c_str(), detection.global_position.x, detection.global_position.y,
                        detection.global_position.z, detection.log_odds);
            }
        }
        object_detections = updated_detections;
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    bool rgb_recv;
    bool pointcloud_recv;
    bool camera_info_recv;
    bool odom_recv;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    
    sensor_msgs::msg::Image::SharedPtr rgb_image_msg;
    sensor_msgs::msg::PointCloud2::SharedPtr pointcloud_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::TimerBase::SharedPtr log_odds_timer;  

    cv_bridge::CvImagePtr rgb_image_ptr;
    cv::Mat rgb_image;
    bool sim;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
