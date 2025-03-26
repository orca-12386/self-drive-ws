#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <functional>  // Required for std::placeholders
#include <eigen3/Eigen/Dense>

class SplineInterpNode : public rclcpp::Node {
public:
    SplineInterpNode() : Node("spline_interp", rclcpp::NodeOptions{}) {
        RCLCPP_INFO(this->get_logger(), "spline_interp node started");

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local", 10,
            std::bind(&SplineInterpNode::mapCallback, this, std::placeholders::_1));

        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map/yellow/local/interp", 10);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    std::vector<cv::Point2d> extractPoints(const nav_msgs::msg::OccupancyGrid::SharedPtr &map_msg) {
        std::vector<cv::Point2d> points;
        for (int y = 0; y < map_msg->info.height; ++y) {
            for (int x = 0; x < map_msg->info.width; ++x) {
                int index = y * map_msg->info.width + x;
                if (map_msg->data[index] == 100) {
                    points.push_back({x * map_msg->info.resolution + map_msg->info.origin.position.x,
                                      y * map_msg->info.resolution + map_msg->info.origin.position.y});
                }
            }
        }
        return points;
    }



    std::vector<cv::Point2d> FitBSpline(const std::vector<cv::Point2d>& points, double maxDistance = 100, int numInterpolated = 300) {
        if (points.size() < 4) return {}; // Need at least 4 points for B-Spline

        // Filter out points that are too far from their neighbors
        std::vector<cv::Point2d> filteredPoints;
        filteredPoints.push_back(points[0]); // Always keep the first point

        for (size_t i = 1; i < points.size(); ++i) {
            double dist = cv::norm(points[i] - filteredPoints.back());
            if (dist <= maxDistance) {
                filteredPoints.push_back(points[i]);
            }
        }

        // Check if we still have enough points after filtering
        if (filteredPoints.size() < 4) return {};

        Eigen::MatrixXd A(filteredPoints.size(), 4);
        Eigen::VectorXd Bx(filteredPoints.size()), By(filteredPoints.size());
        
        for (size_t i = 0; i < filteredPoints.size(); i++) {
            double t = i / static_cast<double>(filteredPoints.size() - 1);
            A(i, 0) = (1 - t) * (1 - t) * (1 - t);
            A(i, 1) = 3 * (1 - t) * (1 - t) * t;
            A(i, 2) = 3 * (1 - t) * t * t;
            A(i, 3) = t * t * t;
            Bx(i) = filteredPoints[i].x;
            By(i) = filteredPoints[i].y;
        }

        Eigen::VectorXd coefX = A.colPivHouseholderQr().solve(Bx);
        Eigen::VectorXd coefY = A.colPivHouseholderQr().solve(By);

        std::vector<cv::Point2d> interpolatedPoints;
        for (int i = 0; i <= numInterpolated; ++i) {
            double t = i / static_cast<double>(numInterpolated);
            double x = coefX(0) * (1 - t) * (1 - t) * (1 - t) +
                    coefX(1) * 3 * (1 - t) * (1 - t) * t +
                    coefX(2) * 3 * (1 - t) * t * t +
                    coefX(3) * t * t * t;
            double y = coefY(0) * (1 - t) * (1 - t) * (1 - t) +
                    coefY(1) * 3 * (1 - t) * (1 - t) * t +
                    coefY(2) * 3 * (1 - t) * t * t +
                    coefY(3) * t * t * t;
            interpolatedPoints.emplace_back(x, y);
        }

        return interpolatedPoints;
    }


    void plotInterpoints(nav_msgs::msg::OccupancyGrid &map_msg, const std::vector<cv::Point2d> &Points, int thickness = 3) {
        cv::Mat map_img(map_msg.info.height, map_msg.info.width, CV_8UC1, cv::Scalar(0));

        for (size_t i = 1; i < Points.size(); i++) {
            cv::Point pt1(
                static_cast<int>((Points[i-1].x - map_msg.info.origin.position.x) / map_msg.info.resolution),
                static_cast<int>((Points[i-1].y - map_msg.info.origin.position.y) / map_msg.info.resolution)
            );
            cv::Point pt2(
                static_cast<int>((Points[i].x - map_msg.info.origin.position.x) / map_msg.info.resolution),
                static_cast<int>((Points[i].y - map_msg.info.origin.position.y) / map_msg.info.resolution)
            );

            cv::line(map_img, pt1, pt2, cv::Scalar(255), thickness);
        }

        for (int y = 0; y < map_msg.info.height; ++y) {
            for (int x = 0; x < map_msg.info.width; ++x) {
                if (map_img.at<uchar>(y, x) > 0) {
                    map_msg.data[y * map_msg.info.width + x] = 100;
                }
            }
        }
    }


    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        auto points = extractPoints(msg);
        auto interpoints = FitBSpline(points, 200);
        auto modified_map = *msg;
        plotInterpoints(modified_map, interpoints);
        map_pub_->publish(modified_map);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SplineInterpNode>());
    rclcpp::shutdown();
    return 0;
}
