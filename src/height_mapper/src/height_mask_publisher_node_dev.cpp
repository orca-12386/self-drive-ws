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
        pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/height_mask/pointcloud", 10);

        timer = this->create_wall_timer(
            std::chrono::milliseconds(40), std::bind(&HeightMaskPublisherNode::timer_callback, this));
    };

private:
    const float pitch = 23.5f * M_PI / 180.0f;
    const float cos_pitch = cos(pitch);
    const float sin_pitch = sin(pitch);
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

    void publish_mask(sensor_msgs::msg::Image::SharedPtr rgb, sensor_msgs::msg::Image::SharedPtr depth, sensor_msgs::msg::CameraInfo::SharedPtr camera_info) {    
        Timer t = Timer("sensor msg to cv mat");
        rgb_image_ptr = cv_bridge::toCvCopy(rgb, rgb->encoding);
        rgb_image = rgb_image_ptr->image;

        depth_image_ptr = cv_bridge::toCvCopy(depth, depth->encoding);
        depth_image = depth_image_ptr->image;

        if (rgb_image.empty() || depth_image.empty()) {
            return;
        } 

        cv::Mat mask_barrel(rgb_image.rows,rgb_image.cols, CV_8U, cv::Scalar(0));
        cv::Mat mask_tyre(rgb_image.rows,rgb_image.cols, CV_8U, cv::Scalar(0));
        cv::Mat mask_mannequin(rgb_image.rows,rgb_image.cols, CV_8U, cv::Scalar(0));
        Point p;
        double depthvalue;

        std::vector<float> points;

        for(int i = 0;i<mask_barrel.rows;i++) {
            for(int j = 0;j<mask_barrel.cols;j++) {
                depthvalue = static_cast<double>(depth_image.at<float>(i,j));
                p = convert_depth_to_point(j, i, depthvalue, camera_info);
                p = cloudPointToBaselink(p);
                if(!(p.x < 15 && p.z > 0.6 && p.z < 1.1)) {
                    mask_barrel.at<uchar>(i,j) = 0;
                } else {
                    mask_barrel.at<uchar>(i,j) = 255;
                    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
                        points.push_back(static_cast<float>(p.x));
                        points.push_back(static_cast<float>(p.y));
                        points.push_back(static_cast<float>(p.z));
                    }
                }
                if (!(p.x < 15 && p.z > 0.1 && p.z < 0.5)) {
                    mask_tyre.at<uchar>(i,j) = 0;
                } else {
                    mask_tyre.at<uchar>(i,j) = 255;
                }
                if (!(p.x < 15 && p.z > 1.1 && p.z < 1.9)) {
                    mask_mannequin.at<uchar>(i,j) = 0;
                } else {
                    mask_mannequin.at<uchar>(i,j) = 255;
                }
            }
        }

        mask_barrel_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_barrel).toImageMsg();
        mask_barrel_pub->publish(*mask_barrel_msg);

        mask_mannequin_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_mannequin).toImageMsg();
        mask_mannequin_pub->publish(*mask_mannequin_msg);

        mask_tyre_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", mask_tyre).toImageMsg();
        mask_tyre_pub->publish(*mask_tyre_msg);

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = rgb->header.stamp;
        cloud_msg.header.frame_id = camera_info->header.frame_id;

        cloud_msg.height = 1;
        cloud_msg.width = points.size() / 3;
        cloud_msg.fields.resize(3);
        cloud_msg.fields[0].name = "x";
        cloud_msg.fields[0].offset = 0;
        cloud_msg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[0].count = 1;
        cloud_msg.fields[1].name = "y";
        cloud_msg.fields[1].offset = 4;
        cloud_msg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[1].count = 1;
        cloud_msg.fields[2].name = "z";
        cloud_msg.fields[2].offset = 8;
        cloud_msg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        cloud_msg.fields[2].count = 1;

        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 12;
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
        cloud_msg.is_dense = false;

        cloud_msg.data.resize(points.size() * sizeof(float));
        memcpy(&cloud_msg.data[0], points.data(), points.size() * sizeof(float));

        pointcloud_pub->publish(cloud_msg);
        log("published");
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
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub;

    sensor_msgs::msg::Image::SharedPtr rgb_image_msg, depth_image_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg;
    nav_msgs::msg::Odometry::SharedPtr odom_msg;

    sensor_msgs::msg::Image::SharedPtr mask_barrel_msg;
    sensor_msgs::msg::Image::SharedPtr mask_tyre_msg;
    sensor_msgs::msg::Image::SharedPtr mask_mannequin_msg;

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