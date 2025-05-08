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
    std::vector<std::array<int, 2>> visited_vec;
    std::unordered_set<uint64_t> visited;
    auto hash_coords = [](const std::array<int, 2>& coords) {
        return (static_cast<uint64_t>(coords[0]) << 32) | static_cast<uint64_t>(coords[1]);
    };
    std::queue<std::array<int, 2>> q;
    std::array<int, 2> p;
    std::array<std::array<int, 2>, 4> neighbours;
    q.push(src);
    while(q.size()>0) {
        p = q.front();
        q.pop();
        if(map->data[p[1]*map->info.width + p[0]] > 0) {
            dst = p;
            return true;
        }
        if(sqrt(pow(p[0]-src[0],2) + pow(p[1]-src[1],2)) > 60) {
            return false;
        }
        neighbours[0] = {p[0]+1, p[1]};
        neighbours[1] = {p[0], p[1]+1};
        neighbours[2] = {p[0]-1, p[1]};
        neighbours[3] = {p[0], p[1]-1};
        visited_vec.push_back(p);
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
        if(!map) {
            return false;
        }
        // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::to_string(detection_location[0])+std::string(",")+std::to_string(detection_location[1])).c_str());
        int grid_x = std::round((detection_location[0] - map->info.origin.position.x) / map->info.resolution);
        int grid_y = std::round((detection_location[1] - map->info.origin.position.y) / map->info.resolution);
        grid_coords[0] = grid_x;
        grid_coords[1] = grid_y;
        return (0<=grid_x && grid_x < map->info.width && 0<=grid_y && grid_y < map->info.height);
    }

    bool check_area(const nav_msgs::msg::OccupancyGrid::SharedPtr near_map, const nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map) {
        std::array<int, 2> src;
        bool grid_status = this->get_grid_coords(near_map, src);
        if(!grid_status) {
            RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("detection location ")+std::to_string(detection_location[0])+std::string(",")+std::to_string(detection_location[1])+std::to_string(detection_location[2])).c_str());
            RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("grid coords not in bounds behaviour_manager ")+std::to_string(src[0])+std::string(",")+std::to_string(src[0])).c_str());
            return false;
        }
        bool swd = get_distance_bfs(src, near_map, wd);
        bool syd = get_distance_bfs(src, yellow_map, yd);
        if(swd && syd) {
            this->edge = wd < 0.2;
            // this->current = std::abs(wd-yd) < 0.5;
            this->current = wd < 3.0 && yd < 3.0;
            this->adjacent = wd - yd >= 0.5;
            return true;
        }
        return false;
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
        std::function<std::array<double, 3>()> get_odometry_location_func, std::function<double()> get_odometry_yaw_func,
        std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_near_map_func,
        std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_yellow_map_func        
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
            if(det->validate_distance(this->distance_threshold, get_odometry_location_func()) && !det->action) {
                det->action = true;
                // RCLCPP_INFO(node->get_logger(), "Returned valid detection");
                return true;
            }
        }
        return false;
    }

private:
    void detectionCallback(geometry_msgs::msg::Point::SharedPtr msg) {
        // RCLCPP_INFO(node->get_logger(), "Received detection");
        double yaw = get_odometry_yaw_func();
        
        std::array<double, 3> odometry_location = get_odometry_location_func();

        std::array<double, 3> location = {msg->x, msg->z, msg->y};
        // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("location: ")+std::to_string(location[f])+std::string(",")+std::to_string(location[1])+std::string(",")+std::to_string(location[2])).c_str());
        rotate_coords(location, yaw);
        // RCLCPP_INFO(rclcpp::get_logger("behaviour_manager"), (std::string("rotated location: ")+std::to_string(location[0])+std::string(",")+std::to_string(location[1])+std::string(",")+std::to_string(location[2])).c_str());
        location = {location[0] + odometry_location[0], location[1] + odometry_location[1], location[2] + odometry_location[2]};
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
            // RCLCPP_INFO(node->get_logger(), "Same");
            for(int i = 0;i<3;i++) {
                prev_detection->detection_location[i] = (alpha*det->detection_location[i]) + ((1-alpha)*prev_detection->detection_location[i]);
            }
            prev_detection->robot_location = odometry_location;
            prev_detection->distance = prev_detection->calculate_distance(odometry_location);
        }
        // if(!is_in_detection_range) {
        //     RCLCPP_INFO(node->get_logger(), "not in range");
        // }

        bool conditions = true;
        conditions = conditions && is_not_previous_detection;
        conditions = conditions && is_in_detection_range;
        if (!conditions) {
            return;
        }
        nav_msgs::msg::OccupancyGrid::SharedPtr near_map = get_near_map_func();
        nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map = get_yellow_map_func();
        if(!near_map || !yellow_map) {
            return;
        }
        bool area_status = det->check_area(near_map, yellow_map);
        if(!area_status) {
            return;
        }
        conditions = conditions && (det->current || det->edge);
        // RCLCPP_INFO(node->get_logger(), std::string(std::string("Distance: ")+std::to_string(det->distance)).c_str());
        if(conditions) {
            add_detection(det);
            RCLCPP_INFO(node->get_logger(), (std::string("Added new detection: ") + topic).c_str());
            // if(prev_detection_exists) {
            //     RCLCPP_INFO(node->get_logger(), prev_detection->to_string().c_str());
            // }
            RCLCPP_INFO(node->get_logger(), det->to_string().c_str());
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
    std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_near_map_func;
    std::function<nav_msgs::msg::OccupancyGrid::SharedPtr()> get_yellow_map_func;
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

    // void feedback_callback(typename action_goal_handle::SharedPtr,
    //     const std::shared_ptr<const typename interface_type::Feedback> feedback)
    // {

    // }

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
        RCLCPP_INFO(node->get_logger(), (std::string("Service call to ") + std::string(service_topic) + std::string("completed")).c_str());
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
        // Detector subscriptions
        near_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local/near", 10, std::bind(&BehaviourManagerNode::nearMapCallback, this, std::placeholders::_1));

        yellow_map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local/interp", 10, std::bind(&BehaviourManagerNode::yellowMapCallback, this, std::placeholders::_1));

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
            {"traffic_drum", 10},
            {"stop_sign", 5},
            {"pedestrian", 10},
            {"tyre", 10}
        };

        for(auto& topic : detection_topics) {
            detection_subs[topic.first] = std::make_unique<DetectionSubscriber>(
                this, 
                topic.second, 
                detection_distance_limits.at(topic.first), 
                std::bind(&BehaviourManagerNode::get_odometry_location, this), 
                std::bind(&BehaviourManagerNode::get_odometry_yaw, this),
                std::bind(&BehaviourManagerNode::get_near_map, this),
                std::bind(&BehaviourManagerNode::get_yellow_map, this)
            );
        }

        // Actions: Lane change, left turn, right turn
        this->lane_change_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "lane_change");
        this->stop_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "StopAction");
        this->left_turn_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "LeftTurn");
        this->right_turn_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "RightTurn");
        this->straight_turn_action_client = std::make_unique<GoalActionClient<interfaces::action::GoalAction>>(this, "StraightTurn");

        // Services: Start/stop lane follow, change planner topic
        this->change_planner_topic_srv_client = std::make_unique<ServiceClient<topic_remapper::srv::ChangeTopic>>(this, "/topic_remapper/motion_control");
        this->lane_follow_toggle_srv_client = std::make_unique<ServiceClient<interfaces::srv::LaneFollowToggle>>(this, "toggle_lane_follow");
        this->detect_intersection_srv_client = std::make_unique<ServiceClient<intersection_detector::srv::DetectIntersection>>(this, "/detection/intersection");
        this->lane_interpolation_toggle_srv_client = std::make_unique<ServiceClient<example_interfaces::srv::SetBool>>(this, "toggle_lane_interpolation");

        lane_follow_toggle_request = std::make_shared<interfaces::srv::LaneFollowToggle::Request>();
        change_planner_topic_request = std::make_shared<topic_remapper::srv::ChangeTopic::Request>();
        detect_intersection_request = std::make_shared<intersection_detector::srv::DetectIntersection::Request>();
        lane_interpolation_toggle_request = std::make_shared<example_interfaces::srv::SetBool::Request>();

        near_map_recv = false;
        yellow_map_recv = false;
        odometry_recv = false;
        
        /*
        mode 0 : neither 
        mode 1 : lane following
        mode 2 : acting
        */
        mode = 0;
        // straight: 0
        // left: 2
        // right: 1
        turn_sequence = {1,0,2};
        current_turn_index = 0;
    }

    std::array<double, 3> get_odometry_location() {
        std::array<double, 3> odometry_location;
        odometry_location[0] = odometry_location_arr[0];
        odometry_location[1] = odometry_location_arr[2];
        odometry_location[2] = odometry_location_arr[1];
        return odometry_location;
    }

    double get_odometry_yaw() {
        return odometry_yaw;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr get_near_map() {
        return near_map_msg;
    }

    nav_msgs::msg::OccupancyGrid::SharedPtr get_yellow_map() {
        return yellow_map_msg;
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


    void lane_interp_toggle(bool toggle) {
        lane_interpolation_toggle_request->data = toggle;            
        lane_interpolation_toggle_srv_client->call(lane_interpolation_toggle_request);
        lane_interpolation_toggle_srv_client->wait_for_result();
        if(toggle) {
            log("Started lane interpolation");
        } else {
            log("Stopped lane interpolation");
        }    
    }

    void lane_follow_toggle(bool toggle) {
        if (toggle) {
            mode = 1;
        } else {
            mode = 0;
        }
        lane_follow_toggle_request->toggle = toggle;            
        lane_follow_toggle_srv_client->call(lane_follow_toggle_request);
        lane_follow_toggle_srv_client->wait_for_result();
        if(toggle) {
            log("Started lane follow");
        } else {
            log("Stopped lane follow");
        }
        lane_interp_toggle(toggle);
    }

    void change_motion_control_map(std::string s) {
        change_planner_topic_request->new_topic = s;
        change_planner_topic_srv_client->call(change_planner_topic_request);
        change_planner_topic_srv_client->wait_for_result();
        log(std::string("Changed planner map to ")+s);
    }

    void lane_change() {
        mode = 2;
        // change planner map to white
        change_motion_control_map("/map/white/local");
        //lane change
        log("Changing lanes");
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        lane_change_action_client->send_goal(goal_msg, send_goal_options);
        lane_change_action_client->wait_for_result();
        log("Changed lanes");
        // change planner map back
        change_motion_control_map("/map/current");
        mode = 0;
    }

    void stop_movement() {
        mode = 2;
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        stop_action_client->send_goal(goal_msg, send_goal_options);
        stop_action_client->wait_for_result();
        mode = 0;
    }

    bool detect_intersection() {
        detect_intersection_srv_client->call(detect_intersection_request);
        detect_intersection_srv_client->wait_for_result();
        auto result = detect_intersection_srv_client->get_result();
        bool detected = result.get()->is_intersection;
        return detected;
    }

    void left_turn() {
        mode = 2;
        change_motion_control_map("/map");
        log("Turning left");
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        left_turn_action_client->send_goal(goal_msg, send_goal_options);
        left_turn_action_client->wait_for_result();
        log("Turned left");
        change_motion_control_map("/map/current");
        mode = 0;
    }

    void right_turn() {
        mode = 2;
        change_motion_control_map("/map");
        log("Turning right");
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        right_turn_action_client->send_goal(goal_msg, send_goal_options);
        right_turn_action_client->wait_for_result();
        log("Turned right");
        change_motion_control_map("/map/current");
        mode = 0;
    }

    void straight_turn() {
        mode = 2;
        change_motion_control_map("/map");
        log("Turning straight");
        auto send_goal_options = create_send_goal_options();
        auto goal_msg = create_goal_message(1);
        straight_turn_action_client->send_goal(goal_msg, send_goal_options);
        straight_turn_action_client->wait_for_result();
        log("Turned straight");
        change_motion_control_map("/map/current");
        mode = 0;
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
        
        if(is_detected.at("stop_sign")) {
            log("Stop sign detected");
            if(detections["stop_sign"]->edge) {
                log("Stop sign detected at edge");
                // stop following
                lane_follow_toggle(false);
                // stop
                stop_movement();
                if(detect_intersection()) {
                    switch(turn_sequence[current_turn_index]) {
                        case 0:
                            straight_turn();
                            break;
                        case 1:
                            right_turn();
                            break;
                        case 2:
                            left_turn();
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
                // stop following
                lane_follow_toggle(false);
                // change planner map
                lane_change();    
            }
        } else if(is_detected.at("traffic_drum")) {
            if(detections["traffic_drum"]->current) {
                log("Traffic drum detected in current lane");
                // stop following
                lane_follow_toggle(false);
                // lane change
                lane_change();    
            } else if(detections["traffic_drum"]->edge) {
                log("Traffic drum detected at edge");
            }
        } else {
            // lane follow
            if(mode == 0) {
                lane_follow_toggle(true);
            }
        }        
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

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lkd_pub;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr yellow_map_msg, near_map_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;

    rclcpp::TimerBase::SharedPtr timer;

    std::array<double, 3> odometry_location_arr;
    double odometry_yaw;

    std::unique_ptr<GoalActionClient<interfaces::action::GoalAction>> lane_change_action_client, stop_action_client, left_turn_action_client, right_turn_action_client, straight_turn_action_client;

    std::unique_ptr<ServiceClient<interfaces::srv::LaneFollowToggle>> lane_follow_toggle_srv_client;
    std::unique_ptr<ServiceClient<topic_remapper::srv::ChangeTopic>> change_planner_topic_srv_client;
    std::unique_ptr<ServiceClient<intersection_detector::srv::DetectIntersection>> detect_intersection_srv_client;
    std::unique_ptr<ServiceClient<example_interfaces::srv::SetBool>> lane_interpolation_toggle_srv_client;

    std::shared_ptr<interfaces::srv::LaneFollowToggle::Request> lane_follow_toggle_request;
    std::shared_ptr<topic_remapper::srv::ChangeTopic::Request> change_planner_topic_request;
    std::shared_ptr<intersection_detector::srv::DetectIntersection::Request> detect_intersection_request;
    std::shared_ptr<example_interfaces::srv::SetBool::Request> lane_interpolation_toggle_request;

    rclcpp::CallbackGroup::SharedPtr timer_cb_group;

    int mode;
    
    int current_turn_index;
    std::array<int, 3> turn_sequence;
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
