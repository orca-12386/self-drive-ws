// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>
#include <map>


class Detection {
public:
    Detection(std::array<double, 3> detection_location, std::array<double, 3> robot_location) {
        this->detection_location = detection_location;
        this->robot_location = robot_location;
        this->distance = calculate_distance(robot_location);
    }

    double calculate_distance(std::array<double, 3> reference_location) {
        double dist = 0;
        for(int i = 0; i<2 ;i++) {
            dist += pow(detection_location[i] - reference_location[i], 2);
        }
        return sqrt(dist);
    }

    bool validate_distance(double threshold, std::array<double, 3> reference_location) {
        double dist = calculate_distance(reference_location);
        return dist <= threshold;
    }
    
    double distance;
    std::array<double, 3> robot_location;
    std::array<double, 3> detection_location;
};

class DetectionSubscriber {
public:
    DetectionSubscriber(rclcpp::Node* node, std::string topic, double distance_threshold, std::function<std::array<double, 3>()> get_odometry_location_func, std::function<double()> get_odometry_yaw_func) {
        this->get_odometry_location_func = get_odometry_location_func;
        this->get_odometry_yaw_func = get_odometry_yaw_func;
        this->node = node;
        this->sub = node->create_subscription<geometry_msgs::msg::Point>(
            topic, 10, std::bind(&DetectionSubscriber::detectionCallback, this, std::placeholders::_1));
        recv = {false, false};
        this->distance_threshold = distance_threshold;
    }

    bool is_detected() {
        Detection* det;
        bool status = this->get_latest_detection(det);
        if(status) {
            if(det->validate_distance(this->distance_threshold, get_odometry_location_func())) {
                return true;
            }
        }
        return false;
    }

private:
    void detectionCallback(geometry_msgs::msg::Point::SharedPtr msg) {
        RCLCPP_INFO(node->get_logger(), "Received detection");
        double yaw = get_odometry_yaw_func();
        std::array<double, 3> odometry_location = get_odometry_location_func();
        std::array<double, 3> location = {msg->x, msg->z, msg->y};
        rotate_coords(location, yaw);
        location = {location[0] + odometry_location[0], location[1] + odometry_location[1], location[2] + odometry_location[2]};
        Detection* det = new Detection(location, odometry_location);

        Detection* prev_detection;
        bool prev_detection_exists = get_latest_detection(prev_detection);
        bool is_not_previous_detection = true;
        if(prev_detection_exists) {
            is_not_previous_detection = !(det->validate_distance(1, prev_detection->detection_location));
        }

        bool is_in_detection_range = det->validate_distance(this->distance_threshold, odometry_location);
        
        if(!is_not_previous_detection) {
            RCLCPP_INFO(node->get_logger(), "Same");
        }
        if(!is_in_detection_range) {
            RCLCPP_INFO(node->get_logger(), "not in range");
        }

        bool conditions = true;
        conditions = conditions && is_not_previous_detection;
        conditions = conditions && is_in_detection_range;
        RCLCPP_INFO(node->get_logger(), std::string(std::string("Distance: ")+std::to_string(det->distance)).c_str());
        if(conditions) {
            add_detection(det);
            RCLCPP_INFO(node->get_logger(), "Added");
        }
    }
    
    void rotate_coords(std::array<double, 3>& p, double yaw) {
        double py = (p[1]*sin(yaw)) - (p[0]*cos(yaw)); 
        double px = (p[0]*sin(yaw)) + (p[1]*cos(yaw));
        p[0] = px;
        p[1] = py;
    }

    void add_detection(Detection*& det) {
        if(recv[1]) {
            if(recv[0]) {
                delete detections[0];
            } else {
                recv[0] = true;
            }
            detections[0] = detections[1];
        } else {
            recv[1] = true;
        }
        detections[1] = det;
    }

    bool get_latest_detection(Detection*& det) {
        if(recv[1]) {
            det = detections[1];
            return true;
        }
        return false;
    }

    bool get_previous_detection(Detection*& det) {
        if(recv[0]) {
            det = detections[0];
            return true;
        }
        return false;
    }

    double distance_threshold;
    std::array<bool, 2> recv;
    std::function<std::array<double, 3>()> get_odometry_location_func;
    std::function<double()> get_odometry_yaw_func;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub;
    std::array<Detection*, 2> detections;
    rclcpp::Node* node;
};
    

class BehaviourManagerNode : public rclcpp::Node
{
public:
    BehaviourManagerNode() : rclcpp::Node("behaviour_manager_node") {
        RCLCPP_INFO(this->get_logger(), "behaviour_manager_node started");

        this->initialise_data();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&BehaviourManagerNode::timer_callback, this));
    };

private:
    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    void initialise_data() {
        // Detector subscriptions
        // near_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        //     "/map/white/local/near", 10, std::bind(&BehaviourManagerNode::nearMapCallback, this, std::placeholders::_1));

        // far_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        //     "/map/white/local/far", 10, std::bind(&BehaviourManagerNode::farMapCallback, this, std::placeholders::_1));

        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&BehaviourManagerNode::odomCallback, this, std::placeholders::_1));

        lkd_pub = this->create_publisher<std_msgs::msg::Bool>("lane_keep_disable", 10);
        
        detection_topics = {
            {"traffic_drum", "/detector/traffic_drum/coordinates"},
            {"stop_sign", "/detector/stop_sign/coordinates"},
            {"pedestrian", "/detector/pedestrian/coordinates"},
            {"tyre", "/detector/tyre/coordinates"}
        };

        detection_distance_limits = {
            {"traffic_drum", 5},
            {"stop_sign", 5},
            {"pedestrian", 5},
            {"tyre", 5}
        };

        for(auto& topic : detection_topics) {
            detection_subs[topic.first] = std::make_unique<DetectionSubscriber>(this, topic.second, detection_distance_limits.at(topic.first), std::bind(&BehaviourManagerNode::get_odometry_location, this), std::bind(&BehaviourManagerNode::get_odometry_yaw, this));
        }
        
        near_map_recv = false;
        far_map_recv = false;
        odometry_recv = false;
    }

    std::array<double, 3> get_odometry_location() {
        return odometry_location_arr;
    }

    double get_odometry_yaw() {
        return odometry_yaw;
    }

    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        this->odometry_location_arr = {odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.z, odometry_msg->pose.pose.position.y};
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        this->odometry_yaw = yaw;
        this->odometry_recv = true;
    }

    // bool get_nearest_point_bfs(const std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<int, 2>& dst) {
    //     std::vector<std::array<int, 2>> visited_vec;
    //     std::unordered_set<uint64_t> visited;
    //     auto hash_coords = [](const std::array<int, 2>& coords) {
    //         return (static_cast<uint64_t>(coords[0]) << 32) | static_cast<uint64_t>(coords[1]);
    //     };
    //     std::queue<std::array<int, 2>> q;
    //     std::array<int, 2> p;
    //     std::array<std::array<int, 2>, 4> neighbours;
    //     q.push(src);
    //     while(q.size()>0) {
    //         p = q.front();
    //         q.pop();
    //         if(map->data[p[1]*map->info.width + p[0]] > 0) {
    //             dst = p;
    //             return true;
    //         }
    //         if(sqrt(pow(p[0]-src[0],2) + pow(p[1]-src[1],2)) > 60) {
    //             return false;
    //         }
    //         neighbours[0] = {p[0]+1, p[1]};
    //         neighbours[1] = {p[0], p[1]+1};
    //         neighbours[2] = {p[0]-1, p[1]};
    //         neighbours[3] = {p[0], p[1]-1};
    //         visited_vec.push_back(p);
    //         for(int i=0;i<4;i++) {
    //             if(neighbours[i][0] >= map->info.width || neighbours[i][0] < 0) {
    //                 continue;
    //             }
    //             if(neighbours[i][1] >= map->info.height || neighbours[i][1] < 0) {
    //                 continue;
    //             }
    //             if(visited.find(hash_coords(neighbours[i])) == visited.end()) {
    //                 q.push(neighbours[i]);
    //                 visited.insert(hash_coords(neighbours[i]));
    //             }
    //         }
    //     }
    //     return false;
    // }

    void process_detections() {
        std::unordered_map<std::string, bool> is_detected;
        for(const auto & topic : detection_subs) {
            is_detected[topic.first] = topic.second->is_detected();
        }
        if(is_detected.at("stop_sign")) {
            log("Stop sign detected");
        } else if(is_detected.at("tyre")) {
            log("Tyre detected");
        } else if(is_detected.at("traffic_drum")) {
            log("Traffic drum detected");
        } else {
            log("no detections");
        }
        // obtain latest detections
        
/*        
if(stop)
automatic: stop keeping, set goal to current position. Wait for cmd vel to become 0 and then continue execution 
manual: stop
If(tyre)
lane change
if(same lane barrel)
automatic: lane change 
manual: stop
if(same lane pedestrian)
if(adjacent lane barrel)
stop and wait for 2s then continue
else 
lane change
if(left turn)
turn left
if(right turn)
turn right
if(parking)
parking
else
keep
*/
    }

    void timer_callback() {
        bool recv = true;
        // recv = recv && near_map_recv;
        // recv = recv && far_map_recv;
        recv = recv && odometry_recv;
        if(recv) {
            process_detections();
        } else {
            log("Waiting for subscriptions");
        }
    }

    std::unordered_map<std::string, double> detection_distance_limits;
    std::unordered_map<std::string, std::string> detection_topics;
    std::unordered_map<std::string, std::unique_ptr<DetectionSubscriber>> detection_subs;

    bool odometry_recv;
    bool near_map_recv, far_map_recv;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr near_map_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr far_map_sub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lkd_pub;


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, nearest_map_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;

    rclcpp::TimerBase::SharedPtr timer;

    std::array<double, 3> odometry_location_arr;
    double odometry_yaw;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviourManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
