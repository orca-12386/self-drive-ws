// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <interfaces/action/goal_action.hpp>
#include <interfaces/srv/lane_follow_toggle.hpp>
#include <topic_remapper/srv/change_topic.hpp>
#include <intersection_detector/srv/detect_intersection.hpp>
#include <example_interfaces/srv/set_bool.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>
#include <map>
#include <unordered_set>
#include <functional>

#include "utils.cpp"


bool convert_to_grid(const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<double, 3> world_coords, std::array<int, 2>& grid_coords) {
    if(!map) {
        return false;
    }
    // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::to_string(detection_location[0])+std::string(",")+std::to_string(detection_location[1])).c_str());
    int grid_x = std::round((world_coords[0] - map->info.origin.position.x) / map->info.resolution);
    int grid_y = std::round((world_coords[1] - map->info.origin.position.y) / map->info.resolution);
    grid_coords[0] = grid_x;
    grid_coords[1] = grid_y;
    return (0<=grid_x && grid_x < map->info.width && 0<=grid_y && grid_y < map->info.height);
}

namespace LaneChecker {
    Map current_map;
    BotPose current_pose;
    BotPose prev_pose;
    bool got_map;
    bool got_odom;

    void update_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        got_map = true;

        current_map.height = msg->info.height;
        current_map.width = msg->info.width;
        current_map.resolution = msg->info.resolution;
        current_map.origin.x = msg->info.origin.position.x;
        current_map.origin.y = msg->info.origin.position.y;

        current_map.grid.resize(current_map.height,
                                std::vector<int>(current_map.width, -1));

        for (int y = 0; y < current_map.height; y++) {
            for (int x = 0; x < current_map.width; x++) {
                int index = y * current_map.width + x;
                current_map.grid[y][x] = msg->data[index];
            }
        }
    }

    void update_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose.world_pose.x = msg->pose.pose.position.x;
        current_pose.world_pose.y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        current_pose.roll = roll;
        current_pose.pitch = pitch;
        current_pose.yaw = yaw;

        if (got_map) {
            current_pose.map_pose =
            Utils::getMapPoseFromWorldPose(current_pose.world_pose, current_map);
        }

        if (!got_odom || Utils::worldDistance(current_pose.world_pose, prev_pose.world_pose)) {
            prev_pose = current_pose;
        }

        got_odom = true;
    }

    bool checkInLane(WorldPose obs_pose) {
        double theta;
        if (Utils::worldDistance(current_pose.world_pose, prev_pose.world_pose) <
            0.1) {
            theta = current_pose.yaw;
        } else {
            theta =
                Utils::getAngleRadians(prev_pose.world_pose, current_pose.world_pose);
        }

        Utils::removeMapBehindBot(current_map, current_pose.world_pose, theta, 20, 20);

        MapPose nearest_lane_mp = Utils::findClosestForValue(current_pose.map_pose, current_map, 300, 100);
        MapPose farthest_lane_mp = Utils::exploreLane(nearest_lane_mp, current_map, 100);

        WorldPose pt1 = Utils::getWorldPoseFromMapPose(nearest_lane_mp, current_map);
        WorldPose pt2 = Utils::getWorldPoseFromMapPose(farthest_lane_mp, current_map);

        // publishMarkers(pt1, pt2);

        float dx = pt2.x - pt1.x;
        float dy = pt2.y - pt1.y;

        // Line equation: Ax + By + C = 0
        float A = dy;
        float B = -dx;
        float C = dx * pt1.y - dy * pt1.x;

        float d1 = std::abs(A * obs_pose.x + B * obs_pose.y + C) / std::sqrt(A * A + B * B);
        if (d1 < 1) {
            // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), "too close to the lane");
            return false;   
        }

        float d2 = std::abs(A * current_pose.world_pose.x + B * current_pose.world_pose.y + C) / std::sqrt(A * A + B * B);

        // Side of the line (sign of the equation without abs)
        float side1 = A * current_pose.world_pose.x + B * current_pose.world_pose.y + C;
        float side2 = A * current_pose.world_pose.x + B * current_pose.world_pose.y + C;

        bool close_enough = std::abs(d1 - d2) <= 2.0;
        bool same_side = (side1 * side2) >= 0;  // both same sign or zero

        // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), "d1: %.2f, d2: %.2f, same_side: %s", d1, d2, same_side ? "true" : "false");

        return close_enough && same_side;
    }
}

namespace LogOdds {
    double hit_prob;
    double miss_prob;
    double mark_prob;
    double unmark_prob;
    double unknown_prob;

    double hit_log_odds;
    double miss_log_odds;
    double mark_log_odds;
    double unmark_log_odds;
    double unknown_log_odds;

    double max_prob;
    double min_prob;
    double max_log_odds;
    double min_log_odds;

    double prob_to_log_odds(double prob) {
        double ratio = (prob/(1-prob));
        return std::log(ratio);
    }

    void initialise() {
        hit_prob = 0.8;
        miss_prob = 0.2;
        mark_prob = 0.85;
        unmark_prob = 0.15;
        unknown_prob = 0.5;
        // max_prob = 0.99;
        // min_prob = 0.12;

        hit_log_odds = prob_to_log_odds(hit_prob);
        miss_log_odds = prob_to_log_odds(miss_prob);
        mark_log_odds = prob_to_log_odds(mark_prob);
        unmark_log_odds = prob_to_log_odds(unmark_prob);
        unknown_log_odds = prob_to_log_odds(unknown_prob);
        max_log_odds = 8;
        min_log_odds = -8;
    }

    bool is_positive(double log_odds) {
        return log_odds > mark_log_odds;
    }

    bool is_negative(double log_odds) {
        return log_odds < unmark_log_odds;
    }

    void clamp(double& log_odds) {
        if(log_odds > max_log_odds) {
            log_odds = max_log_odds;
        }
        if(log_odds < min_log_odds) {
            log_odds = min_log_odds;
        }
    }
}


class Detection {
public:
    Detection(std::array<double, 3> detection_location, std::array<double, 3> robot_location) {
        this->detection_location = detection_location;
        this->robot_location = robot_location;
        this->distance = calculate_distance(robot_location);
        this->action = false;
        this->current = false;
        this->outside = false;
        this->current_log_odds = LogOdds::unknown_log_odds;
    }

    std::string to_string() {
        std::string o0(std::string("World Coordinates: ")+std::to_string(detection_location[0])+std::string(",")+std::to_string(detection_location[1])+std::string(",")+std::to_string(detection_location[2]));
        std::string o1(std::string("Distance: ")+std::to_string(this->distance));
        std::string o2(std::string("Action: ")+std::to_string(this->action));
        std::string o3(std::string("Current: ")+std::to_string(this->current));
        std::string o4(std::string("Outside: ")+std::to_string(this->outside));
        std::string nl("\n");
        return o0+nl+o1+nl+o2+nl+o3+nl+o4+nl;
    }

    void set_locations(std::array<double, 3> detection_location, std::array<double, 3> robot_location) {
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

    bool validate_difference(double threshold, std::array<double, 3> reference_location) {
        bool valid = true;
        double diff;
        for(int i = 0; i<2 ;i++) {
            diff = std::abs(detection_location[i] - reference_location[i]);
            valid = valid && (diff <= threshold);
            // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), std::to_string(diff).c_str());
        }
        return valid;
    }
 
    bool get_grid_coords(const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<int, 2>& grid_coords) {
        return convert_to_grid(map, detection_location, grid_coords);
    }

    void check_area() {
        WorldPose obs_pos = WorldPose(detection_location[0], detection_location[1]);
        bool pred = LaneChecker::checkInLane(obs_pos);
        if(pred) {
            current_log_odds += LogOdds::hit_log_odds;
        } else {
            current_log_odds += LogOdds::miss_log_odds;
        }
        this->current = LogOdds::is_positive(current_log_odds);
        this->outside = LogOdds::is_negative(current_log_odds);
        LogOdds::clamp(current_log_odds);
    }
    
    double current_log_odds;
    bool current;
    bool outside;
    bool action;
    double distance;
    std::array<double, 3> robot_location;
    std::array<double, 3> detection_location;
};


class DetectionSubscriber {
public:
    DetectionSubscriber(rclcpp::Node* node, std::string topic, double distance_threshold, 
        std::function<bool(std::array<double, 3>&)> get_odometry_location_func, std::function<bool(double&)> get_odometry_yaw_func,
        std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_near_map_func,
        std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_yellow_map_func,
        std::function<int()> get_mode_func      
    ) {
        client_cb_group = nullptr;
        rclcpp::SubscriptionOptions options;
        options.callback_group = client_cb_group;        
        this->get_odometry_location_func = get_odometry_location_func;
        this->get_odometry_yaw_func = get_odometry_yaw_func;

        this->get_near_map_func = get_near_map_func;
        this->get_yellow_map_func = get_yellow_map_func;

        this->get_mode_func = get_mode_func;

        this->node = node;
        this->topic = topic;
        this->sub = node->create_subscription<geometry_msgs::msg::Point>(topic, 10, std::bind(&DetectionSubscriber::detectionCallback, this, std::placeholders::_1), options);
        recv = {false, false};
        this->distance_threshold = distance_threshold;
    }

    bool is_detected(Detection*& det_out, int check_area_expected = 0) {
        Detection* det;
        bool status = this->get_latest_detection(det);
        if(status) {
            det_out = det;
            // RCLCPP_INFO(node->get_logger(), "Obtained latest detection:");
            // RCLCPP_INFO(node->get_logger(), det->to_string().c_str());
            std::array<double, 3> odometry_location;
            bool odometry_recv = get_odometry_location_func(odometry_location);
            switch(check_area_expected) {
                case 0:
                    // Doesnt matter where obstacle is
                break;
                case 1:
                    // Action is only triggered when obstacle is outside
                    if(!det->outside) {
                        return false;
                    }
                    break;
                case 2:
                    // Action is only triggered when obstacle is inside
                    if(!det->current) {
                        return false;
                    }
                    break;
            }
            if(det->validate_distance(this->distance_threshold, odometry_location) && !det->action) {
                det->action = true;
                // RCLCPP_INFO(node->get_logger(), "Returned valid detection");
                return true;
            }    
        }
        return false;
    }

private:
    void detectionCallback(geometry_msgs::msg::Point::SharedPtr msg) {
        if(get_mode_func() == 2) {
            return;
        }
        
        std::array<double, 3> odometry_location;
        double yaw;
        bool odometry_recv = get_odometry_location_func(odometry_location);
        bool yaw_recv = get_odometry_yaw_func(yaw);
        nav_msgs::msg::OccupancyGrid::SharedPtr near_map = get_near_map_func();
        nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map = get_yellow_map_func();
        
        if(!odometry_recv || !near_map || !yellow_map) {
            return;
        }

        std::array<double, 3> location = {msg->x, msg->y, msg->z};
        auto det = std::make_unique<Detection>(location, odometry_location);

        Detection* prev_detection = nullptr;
        bool prev_detection_exists = get_latest_detection(prev_detection);
        bool is_not_previous_detection = true;
        
        if(prev_detection_exists && prev_detection != nullptr) {
            is_not_previous_detection = !(det->validate_difference(2, prev_detection->detection_location));
        }

        bool is_in_detection_range = det->validate_distance(this->distance_threshold, odometry_location);
        
        if(prev_detection_exists && prev_detection != nullptr && !is_not_previous_detection) {
            double alpha = 0.65;
            RCLCPP_INFO(node->get_logger(), "Updating existing detection");
            for(int i = 0; i < 3; i++) {
                prev_detection->detection_location[i] = (alpha * det->detection_location[i]) + 
                                                    ((1-alpha) * prev_detection->detection_location[i]);
            }
            prev_detection->robot_location = odometry_location;
            prev_detection->distance = prev_detection->calculate_distance(odometry_location);
            if(!prev_detection->action) {
                bool earlier1 = prev_detection->current;
                bool earlier2 = prev_detection->outside;    
                prev_detection->check_area();
                if(earlier1 != prev_detection->current || earlier2 != prev_detection->outside) {
                    RCLCPP_INFO(node->get_logger(), prev_detection->to_string().c_str());
                }    
            }
            return; 
        }
        
        if(!is_in_detection_range) {
            RCLCPP_INFO(node->get_logger(), "Detection not in range");
            return;
        }

        det->check_area();
        RCLCPP_INFO(node->get_logger(), (std::string("Added new detection: ") + topic).c_str());
        RCLCPP_INFO(node->get_logger(), det->to_string().c_str());
        add_detection(std::move(det));
    }

    void rotate_coords(std::array<double, 3>& p, double yaw) {
        double py = (p[1]*sin(yaw)) - (p[0]*cos(yaw)); 
        double px = (p[0]*sin(yaw)) + (p[1]*cos(yaw));
        p[0] = px;
        p[1] = py;
    }

    void add_detection(std::unique_ptr<Detection> det) {
        // FIXED: Proper shifting of detections array
        if(recv[1]) {
            // Move current detection to previous slot (automatically deletes old [0])
            detections[0] = std::move(detections[1]);
            recv[0] = true;
        } else {
            recv[1] = true;
        }
        // Add new detection
        detections[1] = std::move(det);
    }
    
    bool get_latest_detection(Detection*& det) {
        if(recv[1]) {
            det = detections[1].get();  // Get raw pointer for interface
            return true;
        }
        return false;
    }

    bool get_previous_detection(Detection*& det) {
        if(recv[0]) {
            det = detections[0].get();
            return true;
        }
        return false;
    }

    double distance_threshold;
    std::array<bool, 2> recv;
    std::function<bool(std::array<double, 3>&)> get_odometry_location_func;
    std::function<bool(double&)> get_odometry_yaw_func;
    std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_near_map_func;
    std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_yellow_map_func;
    std::function<int()> get_mode_func;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub;
    std::array<std::unique_ptr<Detection>, 2> detections;
    rclcpp::Node* node;
    rclcpp::CallbackGroup::SharedPtr client_cb_group;
    std::string topic;
};
    

template<typename interface_type>
class GoalActionClient {
public:
    using action_goal_handle = typename rclcpp_action::ClientGoalHandle<interface_type>;
    std::shared_future<typename action_goal_handle::SharedPtr> result_future;

    GoalActionClient(rclcpp::Node* node, std::string action_topic) {
        client_cb_group = nullptr;
        this->client_ = rclcpp_action::create_client<interface_type>(node, action_topic, client_cb_group);
        this->node = node;
        result_recv = false;
        this->action_topic = action_topic;
    }

    void send_goal(typename interface_type::Goal& goal_msg, typename rclcpp_action::Client<interface_type>::SendGoalOptions& send_goal_options) {
        if (!client_->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(node->get_logger(), "Action server not available.");
            return;
        }

        result_recv = false;

        send_goal_options.goal_response_callback = std::bind(&GoalActionClient::goal_response_callback, this, std::placeholders::_1);
        // send_goal_options.feedback_callback = std::bind(&GoalActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(&GoalActionClient::result_callback, this, std::placeholders::_1);
        result_future = this->client_->async_send_goal(goal_msg, send_goal_options);
    }

    void wait_for_result() {
        while(!this->check_result_recv()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    bool check_result_recv() {
        return result_recv;
    }

private:
    void goal_response_callback(typename action_goal_handle::SharedPtr future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(node->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(node->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void result_callback(const typename action_goal_handle::WrappedResult & result) {
        result_recv = true;
        RCLCPP_INFO(node->get_logger(), (std::string("Goal action function: ") + std::string(action_topic)).c_str());
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node->get_logger(), "Goal action succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(node->get_logger(), "Unknown result code");
                return;
        }
    }

    bool result_recv;
    rclcpp::Node* node;
    typename rclcpp_action::Client<interface_type>::SharedPtr client_;
    rclcpp::CallbackGroup::SharedPtr client_cb_group;
    std::string action_topic;
};


template<typename interface_type>
class ServiceClient {
public:
    using shared_future = typename rclcpp::Client<interface_type>::SharedFuture; 

    ServiceClient(rclcpp::Node* node, std::string service_topic) {
        this->service_topic = service_topic;
        this->client = node->create_client<interface_type>(service_topic);
        this->node = node;
        result_recv = false;
    }

    void call(std::shared_ptr<typename interface_type::Request> request) {
        RCLCPP_INFO(node->get_logger(), (std::string("Service call function: ") + std::string(service_topic)).c_str());
        while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
            }
            RCLCPP_INFO(node->get_logger(), "service not available, waiting again...");
        }
        RCLCPP_INFO(node->get_logger(), (std::string("Sending service request to ") + std::string(service_topic)).c_str());
        result = client->async_send_request(request);
    }

    void wait_for_result() {
        RCLCPP_INFO(node->get_logger(), (std::string("Waiting for service ") + std::string(service_topic)).c_str());
        result.wait();
        RCLCPP_INFO(node->get_logger(), (std::string("Service call to ") + std::string(service_topic) + std::string(" completed")).c_str());
    }

    shared_future get_result() {
        return result;
    }

private:
    std::string service_topic;
    bool result_recv;
    rclcpp::Node* node;
    typename rclcpp::Client<interface_type>::SharedPtr client;
    rclcpp::CallbackGroup::SharedPtr client_cb_group;
    shared_future result;
};



class Pattern {
public:
    int behaviour_code;
    std::function<void(bool)> lane_follow_srv;
    std::function<void(bool)> lane_interp_srv;
    std::function<void()> stop_in_lane_action;
    std::function<void()> stop_intersection_action;
    std::function<void()> lane_change_action;
    std::function<bool()> check_intersection;
    std::function<void()> left_turn_action;
    std::function<void()> right_turn_action;
    std::function<void()> straight_turn_action;
    std::function<int()> get_mode;
    std::function<void(int)> wait;

    Pattern(
        int behaviour_code_arg,

        std::function<void(bool)> lane_follow_srv_arg,
        std::function<void(bool)> lane_interp_srv_arg,
        std::function<void()> stop_in_lane_action_arg,
        std::function<void()> stop_intersection_action_arg,
        std::function<void()> lane_change_action_arg,
        std::function<bool()> check_intersection_arg,
        std::function<void()> left_turn_action_arg,
        std::function<void()> right_turn_action_arg,
        std::function<void()> straight_turn_action_arg,

        std::function<int()> get_mode_arg,
        std::function<void(int)> wait_arg
    ) :
    behaviour_code(behaviour_code_arg),
    lane_follow_srv(lane_follow_srv_arg),
    lane_interp_srv(lane_interp_srv_arg),
    stop_in_lane_action(stop_in_lane_action_arg),
    stop_intersection_action(stop_intersection_action_arg),
    lane_change_action(lane_change_action_arg),
    check_intersection(check_intersection_arg),
    left_turn_action(left_turn_action_arg),
    right_turn_action(right_turn_action_arg),
    straight_turn_action(straight_turn_action_arg),
    get_mode(get_mode_arg),
    wait(wait_arg)
    {
        pattern_function_map = {
            {0, std::bind(&Pattern::full_course, this)},
            {1, std::bind(&Pattern::lane_keeping, this)},
            {2, std::bind(&Pattern::left_turn, this)},
            {3, std::bind(&Pattern::right_turn, this)},
            {4, std::bind(&Pattern::pothole_detection, this)},
            {5, std::bind(&Pattern::intersection_lane_keeping, this)},
            {6, std::bind(&Pattern::intersection_left_turn, this)},
            {7, std::bind(&Pattern::intersection_right_turn, this)},
            {8, std::bind(&Pattern::unobstructed_static_pedestrian_in_lane_stop, this)},
            {9, std::bind(&Pattern::obstructed_dynamic_pedestrian_in_lane_stop, this)},
            {10, std::bind(&Pattern::static_pedestrian_lane_change, this)},
            {11, std::bind(&Pattern::obstacle_detection_lane_change, this)},
            {12, std::bind(&Pattern::curved_road_lane_keeping, this)},
            {13, std::bind(&Pattern::curved_road_lane_changing, this)},
            {14, std::bind(&Pattern::merging, this)},
            {15, std::bind(&Pattern::parking_pull_in, this)},
            {16, std::bind(&Pattern::parking_pull_out, this)},
            {17, std::bind(&Pattern::parking_parallel, this)},
        };

        pattern_function = pattern_function_map.at(behaviour_code);
    
        done = false;
        action_count = 0;
        switch(behaviour_code) {
            case 0:
                turn_sequence = {1,0,2};
                current_turn_index = 0;
                detection_limits = {
                    {"traffic_drum", 5},
                    {"stop_sign", 3},
                    {"pedestrian", 3},
                    {"tyre", 5},
                    {"pothole", 5}
                };
                break;
            case 1:
                detection_limits = {
                    {"traffic_drum", 2},
                };
                break;
            case 2:
                detection_limits = {
                    {"traffic_drum", 2},
                };
                break;
            case 3:
                detection_limits = {
                    {"traffic_drum", 2},
                };
                break;
            case 4:
                detection_limits = {
                    {"traffic_drum", 2},
                    {"pothole", 5},
                };
                break;
            case 5:
                detection_limits = {
                    {"traffic_drum", 5},
                    {"stop_sign", 4}
                };
                break;
            case 6:
                detection_limits = {
                    {"stop_sign", 4},
                    {"traffic_drum", 5},
                };
                break;
            case 7:
                detection_limits = {
                    {"stop_sign", 4},
                    {"traffic_drum", 2},
                };
                break;
            case 8:
                detection_limits = {
                    {"traffic_drum", 2},
                    {"pedestrian", 5}
                };
                break;
            case 9:
                detection_limits = {
                    {"traffic_drum", 2},
                    {"pedestrian", 5}
                };
                break;
            case 10:
                detection_limits = {
                    {"traffic_drum", 2},
                    {"pedestrian", 5}
                };
                break;
            case 11:
                detection_limits = {
                    {"traffic_drum", 2},
                };
                break;
            case 12:
                detection_limits = {
                    {"traffic_drum", 2},
                };
                break;
            case 13:
                detection_limits = {
                    {"traffic_drum", 2},
                };
                break;
            case 14:
                detection_limits = {};
                break;
            case 15:
                detection_limits = {};
                break;
            
            case 16:
                detection_limits = {};
                break;
            
            case 17:
                detection_limits = {};
                break;
        }
    }
    
    std::unordered_map<std::string, double> get_detection_limits() {
        return detection_limits;
    }

    void execute_pattern(std::unordered_map<std::string, bool> is_detected, std::unordered_map<std::string, Detection*> detections) {
        this->is_detected = is_detected;
        this->detections = detections;
        pattern_function();
    }

private:
    bool done;
    std::unordered_map<int, std::function<void()>> pattern_function_map;
    std::function<void()> pattern_function;

    std::unordered_map<std::string, bool> is_detected;
    std::unordered_map<std::string, Detection*> detections;

    std::unordered_map<std::string, double> detection_limits;

    int current_turn_index;
    std::vector<int> turn_sequence;

    int action_count;


    void log(std::string s) {
        RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), s.c_str());
    }

    bool check_and_increment_action_count(int expected) {
        if(action_count == expected) {
            action_count++;
            return true;
        }
        return false;
    }

    void lane_keeping() {
        /*
        Lane follow
        See traffic_drum and stops at 3 ft from traffic_drum
        Terminate
        */
        if (is_detected.at("traffic_drum")) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void left_turn() {
        if(!done) {
            left_turn_action();
            done = true;
        }
        if(is_detected.at("traffic_drum")) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void right_turn() {
        if(!done) {
            right_turn_action();
            done = true;
        }
        if(is_detected.at("traffic_drum")) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void pothole_detection() {
        /*
        Lane follow
        Sees pothole and lane changes
        Terminate
        */
        if(is_detected.at("pothole") && check_and_increment_action_count(0)) {
            lane_change_action();
            rclcpp::shutdown();
        } else if(is_detected.at("traffic_drum") && check_and_increment_action_count(1)) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void intersection_lane_keeping() {
        if(is_detected.at("stop_sign") && check_and_increment_action_count(0)) {
            stop_intersection_action();
            if(check_intersection()) {
                straight_turn_action();
            }
        } else if(is_detected.at("traffic_drum") && check_and_increment_action_count(1)) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }   
    }

    void intersection_left_turn() {
        if(is_detected.at("stop_sign") && check_and_increment_action_count(0)) {
            stop_intersection_action();
            if(check_intersection()) {
                left_turn_action();
            }
        } else if(is_detected.at("traffic_drum") && check_and_increment_action_count(1)) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void intersection_right_turn() {
        if(is_detected.at("stop_sign") && check_and_increment_action_count(0)) {
            stop_intersection_action();
            if(check_intersection()) {
                right_turn_action();
            }
        } else if(is_detected.at("traffic_drum") && check_and_increment_action_count(1)) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }   
    }

    void unobstructed_static_pedestrian_in_lane_stop() {
        if(is_detected.at("pedestrian")) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void obstructed_dynamic_pedestrian_in_lane_stop() {
        if(is_detected.at("pedestrian") && check_and_increment_action_count(0)) {
            stop_in_lane_action();
            wait(10);
        } else if(is_detected.at("traffic_drum") && check_and_increment_action_count(1)) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void static_pedestrian_lane_change() {
        if(is_detected.at("pedestrian") && check_and_increment_action_count(0)) {
            lane_change_action();
        } else if(is_detected.at("traffic_drum") && check_and_increment_action_count(1)) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void obstacle_detection_lane_change() {
        if(is_detected.at("traffic_drum") && check_and_increment_action_count(0)) {
            lane_change_action();
        } else if(is_detected.at("traffic_drum") && check_and_increment_action_count(1)) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void curved_road_lane_keeping() {
        lane_keeping();
    }

    void curved_road_lane_changing() {
        obstacle_detection_lane_change();
    }

    void merging() {

    }

    void parking_pull_in() {

    }

    void parking_pull_out() {

    }

    void parking_parallel() {

    }

    void full_course() {
        if(is_detected.at("stop_sign")) {
            if(detections["stop_sign"]->outside) {
                if(check_intersection()) {
                    stop_intersection_action();
                    if(current_turn_index >= turn_sequence.size()) {
                        rclcpp::shutdown();
                    }
                    switch(turn_sequence[current_turn_index]) {
                        case 0:
                            straight_turn_action();
                            break;
                        case 1:
                            right_turn_action();
                            break;
                        case 2:
                            left_turn_action();
                            break;
                    }
                    current_turn_index++;
                } else{
                    stop_in_lane_action();
                }
            }
        } else if(is_detected.at("pedestrian")) {
            if(check_intersection()) {
                /* if pedestrian found at intersection change to stop sign detection */
                is_detected["stop_sign"] = true;
                detections["stop_sign"] = detections["pedestrian"];
                detections["stop_sign"]->outside = true;
                is_detected["pedestrian"] = false;
                full_course();
                return;
            }
            if(detections["pedestrian"]->current) {
                stop_in_lane_action();
                wait(10);
            }
        } else if(is_detected.at("traffic_drum")) {
            if(detections["traffic_drum"]->current) {
                lane_change_action();                
            }
        } else if(is_detected.at("pothole")) {
            lane_change_action();
        } else if(is_detected.at("tyre")) {
            lane_change_action();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

};



class BehaviourManagerNode : public rclcpp::Node
{
public:
    BehaviourManagerNode() : rclcpp::Node("behaviour_manager_node") {
        RCLCPP_INFO(this->get_logger(), "behaviour_manager_node started");

        this->initialise_data();

        timer_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
 
        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&BehaviourManagerNode::timer_callback, this), timer_cb_group);
    };


    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

private:
    void initialise_data() {
        this->declare_parameter<int>("behaviour_code", 0); 

        int behaviour_code = this->get_parameter("behaviour_code").as_int();

        pattern = std::make_unique<Pattern>(
            behaviour_code,

            std::bind(&BehaviourManagerNode::lane_follow_srv, this, std::placeholders::_1),
            std::bind(&BehaviourManagerNode::lane_interp_srv, this, std::placeholders::_1),
            std::bind(&BehaviourManagerNode::stop_in_lane_action, this),
            std::bind(&BehaviourManagerNode::stop_intersection_action, this),
            std::bind(&BehaviourManagerNode::lane_change_action, this),
            std::bind(&BehaviourManagerNode::check_intersection, this),
            std::bind(&BehaviourManagerNode::left_turn_action, this),
            std::bind(&BehaviourManagerNode::right_turn_action, this),
            std::bind(&BehaviourManagerNode::straight_turn_action, this),

            std::bind(&BehaviourManagerNode::get_mode, this),
            std::bind(&BehaviourManagerNode::wait, this, std::placeholders::_1)
        );

        // Detector subscriptions
        near_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local/near", 10, std::bind(&BehaviourManagerNode::nearMapCallback, this, std::placeholders::_1));

        yellow_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local/interp", 10, std::bind(&BehaviourManagerNode::yellowMapCallback, this, std::placeholders::_1));

        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&BehaviourManagerNode::odomCallback, this, std::placeholders::_1));
        
        detection_topics = {
            {"traffic_drum", "/detector/traffic_drum/coordinates"},
            {"stop_sign", "/detector/stop_sign/coordinates"},
            {"pedestrian", "/detector/pedestrian/coordinates"},
            {"tyre", "/detector/tyre/coordinates"},
            {"pothole", "/detector/pothole/coordinates"}
        };

        detection_distance_limits = pattern->get_detection_limits();

        // Actions: Lane change, left turn, right turn
        this->lane_change_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "lane_change");
        this->stop_in_lane_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "StopAction");
        this->stop_intersection_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "stop_intersection");

        this->left_turn_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "LeftTurn");
        this->right_turn_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "RightTurn");
        this->straight_turn_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "StraightTurn");

        // Services: Start/stop lane follow, change planner topic
        this->change_planner_topic_srv_client = std::make_unique<ServiceClient<topic_remapper::srv::ChangeTopic>>(this, "/topic_remapper/motion_control");
        this->lane_follow_srv_client = std::make_unique<ServiceClient<interfaces::srv::LaneFollowToggle>>(this, "toggle_lane_follow");
        this->check_intersection_srv_client = std::make_unique<ServiceClient<intersection_detector::srv::DetectIntersection>>(this, "/detection/intersection");
        this->lane_interpolation_toggle_srv_client = std::make_unique<ServiceClient<example_interfaces::srv::SetBool>>(this, "toggle_lane_interpolation");
        this->clear_intersection_srv_client = std::make_unique<ServiceClient<example_interfaces::srv::SetBool>>(this, "clear_intersection");


        lane_follow_srv_request = std::make_shared<interfaces::srv::LaneFollowToggle::Request>();
        change_planner_topic_request = std::make_shared<topic_remapper::srv::ChangeTopic::Request>();
        check_intersection_request = std::make_shared<intersection_detector::srv::DetectIntersection::Request>();
        lane_interpolation_toggle_request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        clear_intersection_srv_request = std::make_shared<example_interfaces::srv::SetBool::Request>();

        near_map_recv = false;
        yellow_map_recv = false;
        odometry_recv = false;
        

        for(auto& topic : detection_topics) {
            if(detection_distance_limits.find(topic.first) != detection_distance_limits.end()) {
                detection_subs[topic.first] = std::make_unique<DetectionSubscriber>(
                    this, 
                    topic.second,
                    detection_distance_limits.at(topic.first),
                    std::bind(&BehaviourManagerNode::get_odometry_location, this, std::placeholders::_1),
                    std::bind(&BehaviourManagerNode::get_odometry_yaw, this, std::placeholders::_1),
                    std::bind(&BehaviourManagerNode::get_near_map, this),
                    std::bind(&BehaviourManagerNode::get_yellow_map, this),
                    std::bind(&BehaviourManagerNode::get_mode, this)
                );
            }
        }

        /*
        Mode
        mode 0 : neither 
        mode 1 : lane following
        mode 2 : action
        */
        mode = 0;
    }

    bool get_odometry_location(std::array<double, 3>& out) {
        // std::array<double, 3> odometry_location;
        // odometry_location[0] = odometry_location_arr[0];
        // odometry_location[1] = odometry_location_arr[2];
        // odometry_location[2] = odometry_location_arr[1];
        out = odometry_location_arr;
        return odometry_recv;
    }

    bool get_odometry_yaw(double& out) {
        out = odometry_yaw;
        return odometry_recv;
    }

    int get_mode() {
        return mode;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr get_near_map() {
        return near_map_msg;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr get_yellow_map() {
        return yellow_map_msg;
    }

    void odomCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        this->odometry_location_arr = {odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z};
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        this->odometry_yaw = yaw;
        this->odometry_recv = true;

        LaneChecker::update_odom(msg);
    }

    void nearMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        near_map_msg = msg;
        near_map_recv = true;
        
        LaneChecker::update_map(msg);
    } 

    void yellowMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        yellow_map_msg = msg;
        yellow_map_recv = true;
    } 

    interfaces::action::GoalAction::Goal create_goal_message(int data) {
        auto goal_msg = interfaces::action::GoalAction::Goal();
        goal_msg.data = data;
        return goal_msg;
    }

    rclcpp_action::Client<interfaces::action::GoalAction>::SendGoalOptions create_send_goal_options() {
        auto send_goal_options = rclcpp_action::Client<interfaces::action::GoalAction>::SendGoalOptions();
        return send_goal_options;
    }

    void wait(int secs) {
        log((std::string("wait ")+std::to_string(secs)).c_str());
        rclcpp::sleep_for(std::chrono::seconds(secs));
    }

    void change_motion_control_map(std::string s) {
        log((std::string("change_motion_control_map ")+s).c_str());
        change_planner_topic_request->new_topic = s;
        change_planner_topic_srv_client->call(change_planner_topic_request);
        change_planner_topic_srv_client->wait_for_result();
        log("Completed change_motion_control_map");
    }

    void lane_interp_srv(bool toggle) {
        return;
        log((std::string("lane_interp_srv ")+std::to_string(toggle)).c_str());
        lane_interpolation_toggle_request->data = toggle;            
        lane_interpolation_toggle_srv_client->call(lane_interpolation_toggle_request);
        lane_interpolation_toggle_srv_client->wait_for_result();
        log("Completed lane_interp_srv");
    }

    void clear_intersection_srv(bool toggle) {
        log((std::string("clear_intersection_srv ")+std::to_string(toggle)).c_str());
        clear_intersection_srv_request->data = toggle;
        clear_intersection_srv_client->call(clear_intersection_srv_request);
        clear_intersection_srv_client->wait_for_result();
        log("Completed clear_intersection_srv");
    }

    void lane_follow_srv(bool toggle) {
        log((std::string("lane_follow_srv ")+std::to_string(toggle)).c_str());
        if (toggle) {
            mode = 1;
        } else {
            mode = 0;
        }
        lane_follow_srv_request->toggle = toggle;            
        lane_follow_srv_client->call(lane_follow_srv_request);
        lane_follow_srv_client->wait_for_result();
        lane_interp_srv(toggle);
        log("Completed lane_follow_srv");
    }

    void stop_in_lane_action() {
        log("stop_in_lane_action");
        lane_follow_srv(false);
        mode = 2;
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        stop_in_lane_action_client->send_goal(goal_msg, send_goal_options);
        stop_in_lane_action_client->wait_for_result();
        mode = 0;
        log("Completed stop_in_lane_action");
    }

    void stop_intersection_action() {
        log("stop_intersection_action");
        lane_follow_srv(false);
        mode = 2;
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        stop_intersection_action_client->send_goal(goal_msg, send_goal_options);
        stop_intersection_action_client->wait_for_result();
        mode = 0;
        log("Completed stop_intersection_action");
    }

    void lane_change_action() {
        log("lane_change_action");
        lane_follow_srv(false);
        mode = 2;
        change_motion_control_map("/map/white/local");
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        lane_change_action_client->send_goal(goal_msg, send_goal_options);
        lane_change_action_client->wait_for_result();
        change_motion_control_map("/map/current");
        mode = 0;
        log("Completed lane_change_action");
    }

    bool check_intersection() {
        log("check_intersection");
        check_intersection_srv_client->call(check_intersection_request);
        check_intersection_srv_client->wait_for_result();
        auto result = check_intersection_srv_client->get_result();
        bool detected = result.get()->is_intersection;
        log((std::string("Completed check_intersection") + std::to_string(detected)).c_str());
        return detected;
    }

    void turn_action(int turn) {
        log((std::string("turn_action ")+std::to_string(turn)).c_str());
        lane_follow_srv(false);
        mode = 2;
        clear_intersection_srv(true);
        change_motion_control_map("/map");
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        switch(turn) {
            case 0:
                log("Turning straight");
                straight_turn_action_client->send_goal(goal_msg, send_goal_options);
                straight_turn_action_client->wait_for_result();
                log("Turned straight");              
                break;
            case 1:
                log("Turning right");
                right_turn_action_client->send_goal(goal_msg, send_goal_options);
                right_turn_action_client->wait_for_result();
                log("Turned right");
                break;
            case 2:
                log("Turning left");
                left_turn_action_client->send_goal(goal_msg, send_goal_options);
                left_turn_action_client->wait_for_result();
                log("Turned left");
                break;
        }
        clear_intersection_srv(false);
        change_motion_control_map("/map/current");
        mode = 0;
        log("Completed turn_action");
    }

    void left_turn_action() {
        turn_action(2);
    }

    void right_turn_action() {
        turn_action(1);
    }

    void straight_turn_action() {
        turn_action(0);
    }

    void process_detections() {
        std::unordered_map<std::string, bool> is_detected;
        std::unordered_map<std::string, Detection*> detections;
        Detection* det;
        // log(std::string("Processing detections"));
        int check_area_expected;
        for(const auto & topic : detection_subs) {
            check_area_expected = 0;
            if(pattern->behaviour_code == 0 || pattern->behaviour_code == 9) {
                if((!topic.first.compare("pedestrian")) || (!topic.first.compare("traffic_drum"))) {
                    check_area_expected = 2;
                } else if(!topic.first.compare("stop_sign")) {
                    check_area_expected = 1;
                }
            }
            is_detected[topic.first] = topic.second->is_detected(det, check_area_expected);
            if(is_detected[topic.first]) {
                log(std::string("detected: ")+std::string(topic.first));
                detections[topic.first] = det;                
            }
        }
        pattern->execute_pattern(is_detected, detections);
    }

    void timer_callback() {
        bool recv = true;
        recv = recv && near_map_recv;
        recv = recv && yellow_map_recv;
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
    bool near_map_recv, yellow_map_recv;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr near_map_sub;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr yellow_map_sub;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map_msg, near_map_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;

    rclcpp::TimerBase::SharedPtr timer;

    std::array<double, 3> odometry_location_arr;
    double odometry_yaw;

    std::unique_ptr<GoalActionClient<interfaces::action::GoalAction>> lane_change_action_client, stop_intersection_action_client, stop_in_lane_action_client, left_turn_action_client, right_turn_action_client, straight_turn_action_client;

    std::unique_ptr<ServiceClient<interfaces::srv::LaneFollowToggle>> lane_follow_srv_client;
    std::unique_ptr<ServiceClient<topic_remapper::srv::ChangeTopic>> change_planner_topic_srv_client;
    std::unique_ptr<ServiceClient<intersection_detector::srv::DetectIntersection>> check_intersection_srv_client;
    std::unique_ptr<ServiceClient<example_interfaces::srv::SetBool>> lane_interpolation_toggle_srv_client, clear_intersection_srv_client;

    std::shared_ptr<interfaces::srv::LaneFollowToggle::Request> lane_follow_srv_request;
    std::shared_ptr<topic_remapper::srv::ChangeTopic::Request> change_planner_topic_request;
    std::shared_ptr<intersection_detector::srv::DetectIntersection::Request> check_intersection_request;
    std::shared_ptr<example_interfaces::srv::SetBool::Request> lane_interpolation_toggle_request;
    std::shared_ptr<example_interfaces::srv::SetBool::Request> clear_intersection_srv_request;

    rclcpp::CallbackGroup::SharedPtr timer_cb_group;

    int mode;
    
    int current_turn_index;
    std::array<int, 3> turn_sequence;

    std::unique_ptr<Pattern> pattern;
};


int main(int argc, char** argv) {
    LogOdds::initialise();
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviourManagerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // rclcpp::spin(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}