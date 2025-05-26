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


/*
Detection

edge
current
adjacent

find distance to white lane, yellow lane
get difference
classify
*/

bool get_nearest_point_bfs(const std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<int, 2>& dst) {
    std::unordered_set<uint64_t> visited;
    auto hash_coords = [](const std::array<int, 2>& coords) {
        return (static_cast<uint64_t>(coords[0]) << 32) | static_cast<uint64_t>(coords[1]);
    };
    std::queue<std::array<int, 2>> q;
    std::array<int, 2> p;
    std::array<std::array<int, 2>, 4> neighbours;
    q.push(src);
    visited.insert(hash_coords(src));
    while(q.size()>0) {
        p = q.front();
        q.pop();
        if(map->data[p[1]*map->info.width + p[0]] > 0) {
            dst = p;
            return true;
        }
        if(sqrt(pow(p[0]-src[0],2) + pow(p[1]-src[1],2)) > 10/map->info.resolution) {
            return false;
        }
        neighbours[0] = {p[0]+1, p[1]};
        neighbours[1] = {p[0], p[1]+1};
        neighbours[2] = {p[0]-1, p[1]};
        neighbours[3] = {p[0], p[1]-1};
        for(int i=0;i<4;i++) {
            if(neighbours[i][0] >= map->info.width || neighbours[i][0] < 0) {
                continue;
            }
            if(neighbours[i][1] >= map->info.height || neighbours[i][1] < 0) {
                continue;
            }
            if(visited.find(hash_coords(neighbours[i])) == visited.end()) {
                q.push(neighbours[i]);
                visited.insert(hash_coords(neighbours[i]));
            }
        }
    }
    return false;
}

double get_distance_bfs(const std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map, double& distance) {
    std::array<int, 2> dst;
    bool status = get_nearest_point_bfs(src, map, dst);
    if(status) {
        distance = std::sqrt(pow(src[0]-dst[0], 2)+pow(src[1]-dst[1], 2)) * map->info.resolution;
    }
    return status;
}

double calculate_slope(std::array<int, 2> p1, std::array<int, 2> p2) {
    if (p2[0]-p1[0] == 0) {
        return 1e9;
    } else {
        return (p2[1]-p1[1])/(p2[0]-p1[0]);
    }
}

bool check_in_between(std::array<int, 2> p0, std::array<int, 2> p1, std::array<int, 2> p2, double slope) {
    double c1 = p1[1] - slope * p1[0];
    double c2 = p2[1] - slope * p2[0];
    double d1 = slope * p0[0] - p0[1] + c1;
    double d2 = slope * p0[0] - p0[1] + c2;
    return d1 * d2 <= 0;
}

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

class Detection {
public:
    Detection(std::array<double, 3> detection_location, std::array<double, 3> robot_location) {
        this->detection_location = detection_location;
        this->robot_location = robot_location;
        this->distance = calculate_distance(robot_location);
        this->action = false;
        this->current = false;
        this->edge = false;
        this->adjacent = false;
    }

    std::string to_string() {
        std::string o0(std::string("World Coordinates: ")+std::to_string(detection_location[0])+std::string(",")+std::to_string(detection_location[1])+std::string(",")+std::to_string(detection_location[2]));
        std::string o1(std::string("Distance: ")+std::to_string(this->distance));
        std::string o2(std::string("Action: ")+std::to_string(this->action));
        std::string o3(std::string("Current: ")+std::to_string(this->current));
        std::string o4(std::string("Adjacent: ")+std::to_string(this->adjacent));
        std::string o5(std::string("Edge: ")+std::to_string(this->edge));
        std::string o6(std::string("Distance to white lane: ")+std::to_string(this->wd));
        std::string o7(std::string("Distance to yellow lane: ")+std::to_string(this->yd));
        std::string nl("\n");
        return o0+nl+o1+nl+o2+nl+o3+nl+o4+nl+o5+nl+o6+nl+o7+nl;
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

    bool check_area(const nav_msgs::msg::OccupancyGrid::SharedPtr near_map, const nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map, std::array<double, 3> odometry_location) {
        std::array<int, 2> detection_grid, p1, p2, p3, odom_grid;
        bool grid_status = this->get_grid_coords(near_map, detection_grid);
        if(!grid_status) {
            RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("detection location ")+std::to_string(detection_location[0])+std::string(",")+std::to_string(detection_location[1])+std::to_string(detection_location[2])).c_str());
            RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("grid coords not in bounds behaviour_manager ")+std::to_string(detection_grid[0])+std::string(",")+std::to_string(detection_grid[1])).c_str());
            return false;
        }
        convert_to_grid(near_map, odometry_location, odom_grid);
        get_nearest_point_bfs(odom_grid, near_map, p1);
        get_nearest_point_bfs(detection_grid, near_map, p2);
        get_nearest_point_bfs(odom_grid, yellow_map, p3);
        double m = calculate_slope(p1, p2);
        this->current = check_in_between(detection_grid, p1, p3, m);

        bool swd = get_distance_bfs(detection_grid, near_map, wd);
        if(swd) {
            if(wd<0.5) {
                this->current = false;
            }            
        }
        this->edge = !this->current;
        return true;
    }
    
    bool edge, adjacent, current;
    double wd, yd;
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
        // client_cb_group = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        client_cb_group = nullptr;
        rclcpp::SubscriptionOptions options;
        options.callback_group = client_cb_group;        
        this->get_odometry_location_func = get_odometry_location_func;
        this->get_odometry_yaw_func = get_odometry_yaw_func;

        this->get_near_map_func = get_near_map_func;
        this->get_yellow_map_func = get_yellow_map_func;

        this->node = node;
        this->topic = topic;
        this->sub = node->create_subscription<geometry_msgs::msg::Point>(topic, 10, std::bind(&DetectionSubscriber::detectionCallback, this, std::placeholders::_1), options);
        recv = {false, false};
        this->distance_threshold = distance_threshold;
    }

    bool is_detected(Detection*& det_out) {
        Detection* det;
        bool status = this->get_latest_detection(det);
        if(status) {
            det_out = det;
            // RCLCPP_INFO(node->get_logger(), "Obtained latest detection:");
            // RCLCPP_INFO(node->get_logger(), det->to_string().c_str());
            std::array<double, 3> odometry_location;
            bool odometry_recv = get_odometry_location_func(odometry_location);
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
        // Transform wrt odometry
        // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("location: ")+std::to_string(location[f])+std::string(",")+std::to_string(location[1])+std::string(",")+std::to_string(location[2])).c_str());
        // rotate_coords(location, yaw);
        // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("rotated location: ")+std::to_string(location[0])+std::string(",")+std::to_string(location[1])+std::string(",")+std::to_string(location[2])).c_str());
        // location = {location[0] + odometry_location[0], location[1] + odometry_location[1], location[2] + odometry_location[2]};
        // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("global location: ")+std::to_string(location[0])+std::string(",")+std::to_string(location[1])+std::string(",")+std::to_string(location[2])).c_str());
        Detection* det = new Detection(location, odometry_location);

        Detection* prev_detection;
        bool prev_detection_exists = get_latest_detection(prev_detection);
        bool is_not_previous_detection = true;
        if(prev_detection_exists) {
            is_not_previous_detection = !(det->validate_difference(2, prev_detection->detection_location));
        }

        bool is_in_detection_range = det->validate_distance(this->distance_threshold, odometry_location);
        
        if(!is_not_previous_detection) {
            double alpha = 0.65;
            RCLCPP_INFO(node->get_logger(), "Same");
            for(int i = 0;i<3;i++) {
                prev_detection->detection_location[i] = (alpha*det->detection_location[i]) + ((1-alpha)*prev_detection->detection_location[i]);
            }
            prev_detection->robot_location = odometry_location;
            prev_detection->distance = prev_detection->calculate_distance(odometry_location);
        }
        if(!is_in_detection_range) {
            RCLCPP_INFO(node->get_logger(), "not in range");
        }

        bool area_status = det->check_area(near_map, yellow_map, odometry_location);
        bool current_or_edge = det->current || det->edge;

        bool conditions = true;
        conditions = conditions && is_not_previous_detection;
        conditions = conditions && area_status;
        conditions = conditions && current_or_edge;

        if(!area_status) {
            RCLCPP_INFO(node->get_logger(), "area_status false");
        }
        if(!current_or_edge) {
            RCLCPP_INFO(node->get_logger(), "edge and current conditions not satisfied");
        }

        if(conditions) {
            add_detection(det);
            RCLCPP_INFO(node->get_logger(), (std::string("Added new detection: ") + topic).c_str());
            // if(prev_detection_exists) {
            //     RCLCPP_INFO(node->get_logger(), prev_detection->to_string().c_str());
            // }
            RCLCPP_INFO(node->get_logger(), det->to_string().c_str());
        } else {
            delete det;
            return;
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
    std::function<bool(std::array<double, 3>&)> get_odometry_location_func;
    std::function<bool(double&)> get_odometry_yaw_func;
    std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_near_map_func;
    std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_yellow_map_func;
    std::function<int()> get_mode_func;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub;
    std::array<Detection*, 2> detections;
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
                    {"barrel", 5},
                    {"stop_sign", 5},
                    {"pedestrian", 5},
                    {"tyre", 5},
                    {"pothole", 5}
                };
                break;
            case 1:
                detection_limits = {
                    {"barrel", 2},
                };
                break;
            case 2:
                detection_limits = {
                    {"barrel", 2},
                };
                break;
            case 3:
                detection_limits = {
                    {"barrel", 2},
                };
                break;
            case 4:
                detection_limits = {
                    {"barrel", 2},
                    {"pothole", 5},
                };
                break;
            case 5:
                detection_limits = {
                    {"barrel", 5},
                    {"stop_sign", 5}
                };
                break;
            case 6:
                detection_limits = {
                    {"stop_sign", 5},
                    {"barrel", 5},
                };
                break;
            case 7:
                detection_limits = {
                    {"stop_sign", 5},
                    {"barrel", 2},
                };
                break;
            case 8:
                detection_limits = {
                    {"barrel", 2},
                    {"pedestrian", 5}
                };
                break;
            case 9:
                detection_limits = {
                    {"barrel", 2},
                    {"pedestrian", 5}
                };
                break;
            case 10:
                detection_limits = {
                    {"barrel", 2},
                    {"pedestrian", 5}
                };
                break;
            case 11:
                detection_limits = {
                    {"barrel", 2},
                };
                break;
            case 12:
                detection_limits = {
                    {"barrel", 2},
                };
                break;
            case 13:
                detection_limits = {
                    {"barrel", 2},
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

    void lane_keeping() {
        /*
        Lane follow
        See barrel and stops at 3 ft from barrel
        Terminate
        */
        if (is_detected.at("barrel")) {
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
        if(is_detected.at("barrel")) {
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
        if(is_detected.at("barrel")) {
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
        if(is_detected.at("pothole") && action_count++ == 0) {
            lane_change_action();
            rclcpp::shutdown();
        } else if(is_detected.at("barrel") && action_count++ == 1) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void intersection_lane_keeping() {
        if(is_detected.at("stop_sign") && action_count++ == 0) {
            stop_intersection_action();
            if(check_intersection()) {
                straight_turn_action();
            }
        } else if(is_detected.at("barrel") && action_count++ == 1) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }   
    }

    void intersection_left_turn() {
        if(is_detected.at("stop_sign") && action_count++ == 0) {
            stop_intersection_action();
            if(check_intersection()) {
                left_turn_action();
            }
        } else if(is_detected.at("barrel") && action_count++ == 1) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void intersection_right_turn() {
        if(is_detected.at("stop_sign") && action_count++ == 0) {
            stop_intersection_action();
            if(check_intersection()) {
                right_turn_action();
            }
        } else if(is_detected.at("barrel") && action_count++ == 1) {
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
        if(is_detected.at("pedestrian") && action_count++ == 0) {
            stop_in_lane_action();
            wait(30);
        } else if(is_detected.at("barrel") && action_count++ == 1) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void static_pedestrian_lane_change() {
        if(is_detected.at("pedestrian") && action_count++ == 0) {
            lane_change_action();
        } else if(is_detected.at("barrel") && action_count++ == 1) {
            stop_in_lane_action();
            rclcpp::shutdown();
        } else {
            if(get_mode() == 0) {
                lane_follow_srv(true);
            }
        }
    }

    void obstacle_detection_lane_change() {
        if(is_detected.at("barrel") && action_count++ == 0) {
            lane_change_action();
        } else if(is_detected.at("barrel") && action_count++ == 1) {
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
            log("Stop sign detected");
            if(detections["stop_sign"]->edge) {
                log("Stop sign detected at edge");
                lane_follow_srv(false);
                stop_in_lane_action();
                if(check_intersection()) {
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
                    if (current_turn_index < turn_sequence.size()-1) {
                        current_turn_index++;
                    }
                }    
            }
        } else if(is_detected.at("tyre")) {
            if(detections["tyre"]->current) {
                log("Tyre detected");
                lane_follow_srv(false);
                lane_change_action();    
            }
        } else if(is_detected.at("barrel")) {
            if(detections["barrel"]->current) {
                log("Traffic drum detected in current lane");
                lane_follow_srv(false);
                lane_change_action();    
            } else if(detections["barrel"]->edge) {
                log("Traffic drum detected at edge");
            }
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

        pattern = new Pattern(
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
            {"barrel", "/detector/barrel/coordinates"},
            {"stop_sign", "/detector/stop_sign/coordinates"},
            {"pedestrian", "/detector/pedestrian/coordinates"},
            {"tyre", "/detector/tyre/coordinates"}
        };

        detection_distance_limits = {
            {"barrel", 5},
            {"stop_sign", 3},
            {"pedestrian", 5},
            {"tyre", 5}
        };

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

        lane_follow_srv_request = std::make_shared<interfaces::srv::LaneFollowToggle::Request>();
        change_planner_topic_request = std::make_shared<topic_remapper::srv::ChangeTopic::Request>();
        check_intersection_request = std::make_shared<intersection_detector::srv::DetectIntersection::Request>();
        lane_interpolation_toggle_request = std::make_shared<example_interfaces::srv::SetBool::Request>();
        clear_intersection_srv_request = std::make_shared<example_interfaces::srv::SetBool::Request>();

        near_map_recv = false;
        yellow_map_recv = false;
        odometry_recv = false;
        

        for(auto& topic : detection_topics) {
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
    }

    void nearMapCallback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        near_map_msg = msg;
        near_map_recv = true;
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
        std::this_thread::sleep_for(std::chrono::seconds(secs));
    }

    void change_motion_control_map(std::string s) {
        change_planner_topic_request->new_topic = s;
        change_planner_topic_srv_client->call(change_planner_topic_request);
        change_planner_topic_srv_client->wait_for_result();
        log(std::string("Changed planner map to ")+s);
    }

    void lane_interp_srv(bool toggle) {
        lane_interpolation_toggle_request->data = toggle;            
        lane_interpolation_toggle_srv_client->call(lane_interpolation_toggle_request);
        lane_interpolation_toggle_srv_client->wait_for_result();
        if(toggle) {
            log("Started lane interpolation");
        } else {
            log("Stopped lane interpolation");
        }    
    }

    void clear_intersection_srv(bool toggle) {
        clear_intersection_srv_request->data = toggle;
        clear_intersection_srv_client->call(clear_intersection_srv_request);
        clear_intersection_srv_client->wait_for_result();
        if(toggle) {
            log("Started intersection clearing");
        } else {
            log("Stopped intersection clearing");
        }
    }

    void lane_follow_srv(bool toggle) {
        if (toggle) {
            mode = 1;
        } else {
            mode = 0;
        }
        lane_follow_srv_request->toggle = toggle;            
        lane_follow_srv_client->call(lane_follow_srv_request);
        lane_follow_srv_client->wait_for_result();
        if(toggle) {
            log("Started lane follow");
        } else {
            log("Stopped lane follow");
        }
        lane_interp_srv(toggle);
    }

    void stop_in_lane_action() {
        lane_follow_srv(false);
        mode = 2;
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        stop_in_lane_action_client->send_goal(goal_msg, send_goal_options);
        stop_in_lane_action_client->wait_for_result();
        mode = 0;
    }

    void stop_intersection_action() {
        lane_follow_srv(false);
        mode = 2;
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        stop_intersection_action_client->send_goal(goal_msg, send_goal_options);
        stop_intersection_action_client->wait_for_result();
        mode = 0;
    }

    void lane_change_action() {
        lane_follow_srv(false);
        mode = 2;
        change_motion_control_map("/map/white/local");
        log("Changing lanes");
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        lane_change_action_client->send_goal(goal_msg, send_goal_options);
        lane_change_action_client->wait_for_result();
        log("Changed lanes");
        change_motion_control_map("/map/current");
        mode = 0;
    }

    bool check_intersection() {
        check_intersection_srv_client->call(check_intersection_request);
        check_intersection_srv_client->wait_for_result();
        auto result = check_intersection_srv_client->get_result();
        bool detected = result.get()->is_intersection;
        return detected;
    }

    void turn_action(int turn) {
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
        for(const auto & topic : detection_subs) {
            is_detected[topic.first] = topic.second->is_detected(det);
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

    Pattern* pattern;
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BehaviourManagerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    // rclcpp::spin(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}