#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>

double distance(cv::Point p1, cv::Point p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}


class LinearInterpNode : public rclcpp::Node {
public:
    LinearInterpNode() : Node("linear_interp", rclcpp::NodeOptions{}) {
        RCLCPP_INFO(this->get_logger(), "linear_interp node started");

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local", 10,
            std::bind(&LinearInterpNode::mapCallback, this, std::placeholders::_1));

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

    void plotLine(nav_msgs::msg::OccupancyGrid &map_msg, const std::vector<cv::Point2d> &Points, int maxdistance = 100){
        cv::Mat map_img(map_msg.info.height, map_msg.info.width, CV_8UC1, cv::Scalar(0));

        for (size_t i = 0; i < Points.size(); i++){
            cv::Point pt1(
                static_cast<int>((Points[i-1].x - map_msg.info.origin.position.x) / map_msg.info.resolution),
                static_cast<int>((Points[i-1].y - map_msg.info.origin.position.y) / map_msg.info.resolution)
            );
            cv::Point pt2(
                static_cast<int>((Points[i].x - map_msg.info.origin.position.x) / map_msg.info.resolution),
                static_cast<int>((Points[i].y - map_msg.info.origin.position.y) / map_msg.info.resolution)
            );
            // std::cout << distance(pt1, pt2) << std::endl ;
            if (distance(pt1, pt2) < maxdistance){
                cv::line(map_img, pt1, pt2, cv::Scalar(255), 2);
                }
        }

        for (int y = 0; y < map_msg.info.height; y++){
            for (int x = 0; x < map_msg.info.width; x++){
                if (map_img.at<uchar>(y, x) > 0){
                    map_msg.data[y * map_msg.info.width + x] = 100;
                }
            }
        }
    }
    void mapCallback (const nav_msgs::msg::OccupancyGrid::SharedPtr map){
        auto pts = extractPoints(map);
        auto p_map = *map;
        plotLine(p_map, pts);
        map_pub_->publish(p_map);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LinearInterpNode>());
    rclcpp::shutdown();
    return 0;
}
