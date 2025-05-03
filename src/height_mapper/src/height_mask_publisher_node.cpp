// #define DEBUG

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
#include <geometry_msgs/msg/point.hpp>  // Changed from point_stamped.hpp
#include <geometry_msgs/msg/pose_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <memory>
#include <string>
#include <cmath>
#include <unordered_map>
#include <algorithm>
#include <vector>

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

// Adding the missing Centroid3D class
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

class HeightMaskPublisherNode : public rclcpp::Node
{
public:
    HeightMaskPublisherNode() : 
    rclcpp::Node("height_mask_publisher_node")
    {
        RCLCPP_INFO(this->get_logger(), "height_mask_publisher_node started");
        rgb_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/image_raw", 10, std::bind(&HeightMaskPublisherNode::rgbImageCallback, this, std::placeholders::_1));
    
        depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
            "/zed_node/stereocamera/depth/image_raw", 10, std::bind(&HeightMaskPublisherNode::depthImageCallback, this, std::placeholders::_1));

        camera_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/zed_node/stereocamera/camera_info", 10, std::bind(&HeightMaskPublisherNode::cameraInfoCallback, this, std::placeholders::_1));
    
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odometry", 10, std::bind(&HeightMaskPublisherNode::odomCallback, this, std::placeholders::_1)
        );
        rgb_recv = false;
        depth_recv = false;
        camera_info_recv = false;
        odom_recv = false;

        mask_barrel_pub = this->create_publisher<sensor_msgs::msg::Image>("/height_mask/barrel", 10);
        mask_mannequin_pub = this->create_publisher<sensor_msgs::msg::Image>("/height_mask/mannequin", 10);
        mask_tyre_pub = this->create_publisher<sensor_msgs::msg::Image>("/height_mask/tyre", 10);
        mask_stop_sign_pub = this->create_publisher<sensor_msgs::msg::Image>("/height_mask/stop_sign", 10);
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mask/pointcloud", 10);

        // Changed from PointStamped to Point publishers
        barrel_centroids_pub = this->create_publisher<geometry_msgs::msg::Point>("/detector/traffic_drum/coordinates", 10);
        mannequin_centroids_pub = this->create_publisher<geometry_msgs::msg::Point>("/detector/pedestrian/coordinates", 10);
        tyre_centroids_pub = this->create_publisher<geometry_msgs::msg::Point>("/detector/tyre/coordinates", 10);
        stop_sign_centroids_pub = this->create_publisher<geometry_msgs::msg::Point>("/detector/stop_sign/coordinates", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&HeightMaskPublisherNode::timer_callback, this));
            
        height_ranges["tyre"] = std::make_pair(0.1f, 0.6f);
        height_ranges["barrel"] = std::make_pair(0.6f, 1.2f);
        height_ranges["mannequin"] = std::make_pair(1.8f, 2.2f);
        height_ranges["stop_sign"] = std::make_pair(1.4f, 1.8f);
    };

private:
    const float pitch = 23.5f * M_PI / 180.0f;
    const float cos_pitch = cos(pitch);
    const float sin_pitch = sin(pitch);
    
    std::unordered_map<std::string, std::pair<float, float>> height_ranges;
    
    void rgbImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->rgb_image_msg = msg;
        rgb_recv = true;
    }

    void depthImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        this->depth_image_msg = msg;
        depth_recv = true;
    }

    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        this->camera_info_msg = msg;
        camera_info_recv = true;
    }
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odom_msg = msg;
        odom_recv = true;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    typedef struct Point {
        double x;
        double y;
        double z;
    } Point;

    Point convert_depth_to_point(double u, double v, const double& depth, const sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {
        double fx = camera_info->k[0]; 
        double fy = camera_info->k[4];
        double cx = camera_info->k[2];
        double cy = camera_info->k[5];
        double z = depth;
        double x = ((u-cx)*z)/fx;
        double y = ((v-cy)*z)/fy;
        Point p = Point();
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }
    
    Point cloudPointToBaselink(Point &cloud_point) {
        Point base_link_point;
        base_link_point.x = cloud_point.z * cos_pitch - cloud_point.y * sin_pitch;
        base_link_point.y = -cloud_point.x;
        base_link_point.z =
            1.5f - cloud_point.z * sin_pitch - cloud_point.y * cos_pitch;
        // base_link_point.rgba = cloud_point.rgba;

        return base_link_point;
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
                
                Point p = convert_depth_to_point(j, i, depthvalue, camera_info);
                Point base_point = cloudPointToBaselink(p);
                
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
                        Point p = convert_depth_to_point(j, i, depthvalue, camera_info);
                        Point base_point = cloudPointToBaselink(p);
                        
                        obstacle_labels.at<int>(i, j) = label;
                        
                        max_obstacle_heights[label] = std::max(max_obstacle_heights[label], static_cast<float>(base_point.z));
                    }
                }
            }
        }
    }

    void publish_mask(sensor_msgs::msg::Image::SharedPtr rgb, sensor_msgs::msg::Image::SharedPtr depth, sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {    
        Timer t = Timer("sensor msg to cv mat");
        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        rgb_image = rgb_image_ptr->image;

        depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        depth_image = depth_image_ptr->image;

        if (rgb_image.empty() || depth_image.empty()) {
            return;
        } 

        cv::Mat mask_barrel(rgb_image.rows, rgb_image.cols, CV_8U, cv::Scalar(0));
        cv::Mat mask_tyre(rgb_image.rows, rgb_image.cols, CV_8U, cv::Scalar(0));
        cv::Mat mask_mannequin(rgb_image.rows, rgb_image.cols, CV_8U, cv::Scalar(0));
        cv::Mat mask_stop_sign(rgb_image.rows, rgb_image.cols, CV_8U, cv::Scalar(0));
        
        cv::Mat obstacle_labels;
        std::vector<float> max_obstacle_heights;
        cv::Mat centroids_mat;
        computeObstacleLabels(depth_image, camera_info, obstacle_labels, max_obstacle_heights, centroids_mat);
        
        std::vector<float> points_barrel;
        std::vector<float> points_mannequin;
        std::vector<float> points_tyre;
        std::vector<float> points_stop_sign;
        
        std::vector<std::string> obstacle_types(max_obstacle_heights.size(), "");
        for (size_t i = 1; i < max_obstacle_heights.size(); i++) {
            float max_height = max_obstacle_heights[i];
            
            // Classify obstacle based on max height
            if (max_height >= height_ranges["tyre"].first && max_height <= height_ranges["tyre"].second) {
                obstacle_types[i] = "tyre";
            } else if (max_height >= height_ranges["barrel"].first && max_height <= height_ranges["barrel"].second) {
                obstacle_types[i] = "barrel";
            } else if (max_height >= height_ranges["mannequin"].first && max_height <= height_ranges["mannequin"].second) {
                obstacle_types[i] = "mannequin";
            } else if (max_height >= height_ranges["stop_sign"].first && max_height <= height_ranges["stop_sign"].second) {
                obstacle_types[i] = "stop_sign";}
        }
        
        std::vector<Centroid3D> barrel_centroids;
        std::vector<Centroid3D> mannequin_centroids;
        std::vector<Centroid3D> tyre_centroids;
        std::vector<Centroid3D> stop_sign_centroids;

        std::unordered_map<int, int> barrel_centroid_indices;
        std::unordered_map<int, int> mannequin_centroid_indices;
        std::unordered_map<int, int> tyre_centroid_indices;
        std::unordered_map<int, int> stop_sign_centroid_indices;


        for (size_t i = 1; i < obstacle_types.size(); i++) {
            if (obstacle_types[i] == "barrel") {
                barrel_centroid_indices[i] = barrel_centroids.size();
                barrel_centroids.push_back(Centroid3D());
            } else if (obstacle_types[i] == "mannequin") {
                mannequin_centroid_indices[i] = mannequin_centroids.size();
                mannequin_centroids.push_back(Centroid3D());
            } else if (obstacle_types[i] == "tyre") {
                tyre_centroid_indices[i] = tyre_centroids.size();
                tyre_centroids.push_back(Centroid3D());
            } else if (obstacle_types[i] == "stop_sign") {
                stop_sign_centroid_indices[i] = stop_sign_centroids.size();
                stop_sign_centroids.push_back(Centroid3D());
            }
        }

        for (int i = 0; i < depth_image.rows; i++) {
            for (int j = 0; j < depth_image.cols; j++) {
                int label = obstacle_labels.at<int>(i, j);
                if (label > 0) { // Skip background
                    double depthvalue = static_cast<double>(depth_image.at<float>(i, j));
                    if (std::isfinite(depthvalue) && depthvalue > 0) {
                        Point p = convert_depth_to_point(j, i, depthvalue, camera_info);
                        Point base_point = cloudPointToBaselink(p);
                        
                        if (obstacle_types[label] == "barrel") {
                            mask_barrel.at<uchar>(i, j) = 255;
                            
                            barrel_centroids[barrel_centroid_indices[label]].addPoint(base_point.x, base_point.y, base_point.z);

                        } else if (obstacle_types[label] == "tyre") {
                            mask_tyre.at<uchar>(i, j) = 255;
                            tyre_centroids[tyre_centroid_indices[label]].addPoint(base_point.x, base_point.y, base_point.z);

                        } else if (obstacle_types[label] == "mannequin") {
                            mask_mannequin.at<uchar>(i, j) = 255;
                            mannequin_centroids[mannequin_centroid_indices[label]].addPoint(base_point.x, base_point.y, base_point.z);
                        } else if (obstacle_types[label] == "stop_sign") {
                            mask_stop_sign.at<uchar>(i, j) = 255;
                            stop_sign_centroids[stop_sign_centroid_indices[label]].addPoint(base_point.x, base_point.y, base_point.z);
                    }
                }
            }
        }
    }
        for (auto& centroid : barrel_centroids) centroid.finalize();
        for (auto& centroid : mannequin_centroids) centroid.finalize();
        for (auto& centroid : tyre_centroids) centroid.finalize();
        for (auto& centroid : stop_sign_centroids) centroid.finalize();

        mask_barrel_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_barrel).toImageMsg();
        mask_barrel_pub->publish(*mask_barrel_msg);

        mask_mannequin_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_mannequin).toImageMsg();
        mask_mannequin_pub->publish(*mask_mannequin_msg);

        mask_tyre_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_tyre).toImageMsg();
        mask_tyre_pub->publish(*mask_tyre_msg);

        mask_stop_sign_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_stop_sign).toImageMsg();
        mask_stop_sign_pub->publish(*mask_stop_sign_msg);
        
        publishCentroids(barrel_centroids, barrel_centroids_pub);
        publishCentroids(mannequin_centroids, mannequin_centroids_pub);
        publishCentroids(tyre_centroids, tyre_centroids_pub);
        publishCentroids(stop_sign_centroids, stop_sign_centroids_pub);

        // sensor_msgs::msg::PointCloud2 cloud_msg;
        // cloud_msg.header.stamp = rgb->header.stamp;
        // cloud_msg.header.frame_id = camera_info->header.frame_id;

        // cloud_msg.height = 1;
        // cloud_msg.width = points.size() / 3;
        // cloud_msg.fields.resize(3);
        // cloud_msg.fields[0].name = "x";
        // cloud_msg.fields[0].offset = 0;
        // cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        // cloud_msg.fields[0].count = 1;
        // cloud_msg.fields[1].name = "y";
        // cloud_msg.fields[1].offset = 4;
        // cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        // cloud_msg.fields[1].count = 1;
        // cloud_msg.fields[2].name = "z";
        // cloud_msg.fields[2].offset = 8;
        // cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        // cloud_msg.fields[2].count = 1;

        // cloud_msg.is_bigendian = false;
        // cloud_msg.point_step = 12;
        // cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        // cloud_msg.is_dense = false;

        // cloud_msg.data.resize(points.size() * sizeof(float));
        // memcpy(&cloud_msg.data[0], points.data(), points.size() * sizeof(float));

        // pointcloud_pub->publish(cloud_msg);
        // log("published");
    }
    
    // Modified to publish Point instead of PointStamped
    void publishCentroids(const std::vector<Centroid3D>& centroids, 
                        rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher) {
        for (const auto& centroid : centroids) {
            if (centroid.point_count > 0) {
                geometry_msgs::msg::Point point_msg;
                point_msg.x = centroid.x;
                point_msg.y = centroid.y;
                point_msg.z = centroid.z;
                publisher->publish(point_msg);
            }
        }
    }

    void timer_callback() {
        bool recv;
        recv = rgb_recv;
        recv = recv && depth_recv;
        recv = recv && camera_info_recv;
        if(recv) {
            publish_mask(rgb_image_msg, depth_image_msg, camera_info_msg);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub, depth_sub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

    bool rgb_recv;
    bool depth_recv;
    bool camera_info_recv;
    bool odom_recv;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_barrel_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_tyre_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_mannequin_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_stop_sign_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;
    
    // Changed from PointStamped to Point
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr barrel_centroids_pub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr mannequin_centroids_pub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr tyre_centroids_pub;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr stop_sign_centroids_pub;

    sensor_msgs::msg::Image::SharedPtr rgb_image_msg, depth_image_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;

    sensor_msgs::msg::Image::SharedPtr mask_barrel_msg;
    sensor_msgs::msg::Image::SharedPtr mask_tyre_msg;
    sensor_msgs::msg::Image::SharedPtr mask_mannequin_msg;
    sensor_msgs::msg::Image::SharedPtr mask_stop_sign_msg;

    rclcpp::TimerBase::SharedPtr timer;

    cv_bridge::CvImagePtr rgb_image_ptr, depth_image_ptr;
    cv::Mat rgb_image, depth_image;
    cv::Mat y_image, height_mask;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HeightMaskPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}