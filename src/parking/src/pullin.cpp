#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <queue>
#include <vector>
#include <utility>
#include <tf2/LinearMath/Quaternion.h>          // For quaternion operations
#include <tf2/LinearMath/Matrix3x3.h>           // For converting quaternion to roll, pitch, yaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For geometry_msgs compatibility with tf2
#include <tf2_ros/buffer.h>                     // For transform buffer
#include <tf2_ros/transform_listener.h>         // For listening to transforms
#include <geometry_msgs/msg/transform_stamped.hpp> // For handling transform messages

const size_t MAX_SIZE = 10;

using namespace std::chrono_literals;

struct GridPoint {
    int x;
    int y;
    
    bool operator==(const GridPoint& other) const {
        return x == other.x && y == other.y;
    }
};

namespace std {
    template<> struct hash<GridPoint> {
        size_t operator()(const GridPoint& p) const {
            return hash<int>()(p.x) ^ (hash<int>()(p.y) << 1);
        }
    };
}

class LaneAnalyzer : public rclcpp::Node {
public:
    LaneAnalyzer() : Node("Pull_In") {

        RCLCPP_INFO(this->get_logger(),"Lane Detector Initialized");
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom/transformed", 10, std::bind(&LaneAnalyzer::odom_callback, this, std::placeholders::_1));
            
        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local", 10,//rclcpp::QoS(10).transient_local(),
            std::bind(&LaneAnalyzer::map_callback, this, std::placeholders::_1));
            
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "lane_markers", 10);
            
        park_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            "parking_markers", 10);
        
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    }

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        current_map_ = msg;
        RCLCPP_INFO(get_logger(), "Received new map");
    }

    std::pair<double, double> get_front_vector(double yaw) {
        return {cos(yaw), sin(yaw)};  // X and Y components of front direction
    }


    void addToHistory(double new_value, std::vector<int> container) {
        if (container.size() >= MAX_SIZE) {
            container.erase(container.begin());  // remove oldest
        }
        container.push_back(new_value);  // add new one
    }

    void general_goal(int goal_x,int goal_y)
    {
        auto world_goal = grid_to_world({goal_x,goal_y});

        geometry_msgs::msg::PoseStamped goal;
        goal.pose.position.x = world_goal.x;
        goal.pose.position.y = world_goal.y;
        goal.header.frame_id = current_map_->header.frame_id;
        tf2::Quaternion quat;

        quat.setRPY(0,0,start_yaw);

        goal.pose.orientation.x = quat.x();
        goal.pose.orientation.y = quat.y();
        goal.pose.orientation.z = quat.z();
        goal.pose.orientation.w = quat.w();
        goal_pub_->publish(goal);
    }
    
    void pub_parking_goal(int goal_x,int goal_y,double yaw)
    {
        int goal_grid_x = goal_x;
        int goal_grid_y = goal_y;


        RCLCPP_WARN(get_logger(),"PARKING DETECTED X: %d. Y %d",goal_grid_x,goal_grid_y);
        GridPoint grid_goal = {goal_grid_x,goal_grid_y};
        auto world_goal = grid_to_world(grid_goal);
        RCLCPP_WARN(get_logger(),"PARKING DETECTED X: %f. Y %f",world_goal.x,world_goal.y);
        geometry_msgs::msg::PoseStamped goal;
        goal.pose.position.x = world_goal.x;
        goal.pose.position.y = world_goal.y;
        goal.header.frame_id = current_map_->header.frame_id;
        double yaw_perpendicular = yaw + 3.14 / 2; //Rotating goal 90 ccw wrt bot's orientation
        tf2::Quaternion quat;
        quat.setRPY(0,0,yaw_perpendicular);

        goal.pose.orientation.x = quat.x();
        goal.pose.orientation.y = quat.y();
        goal.pose.orientation.z = quat.z();
        goal.pose.orientation.w = quat.w();
        while(rclcpp::ok())
        {
            goal_pub_->publish(goal);
        }
    }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!current_map_) return;
        
        // Inside odom_callback after getting current_pos
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);  // Convert ROS msg to tf2 quaternion
        tf2::Matrix3x3 m(q);                          // Create rotation matrix
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);                   // Extract RPY angles
        if(start_yaw == -100)
            start_yaw = yaw;

        GridPoint current_pos = world_to_grid(
            msg->pose.pose.position.x, 
            msg->pose.pose.position.y
        );

        GridPoint ahead_pos = {
            current_pos.x,
            current_pos.y + 20
        };
        
        // Use the position ahead for lane detection
        auto [left_lane, right_lane] = find_adjacent_lanes(ahead_pos, yaw);
        // // Replace existing find_adjacent_lanes call with:
        // auto [left_lane, right_lane] = find_adjacent_lanes(current_pos, yaw);


        // Change detect_parking_spot call to:
        // visualize_orientation(current_pos,yaw);
        if (left_lane.x != -1 && right_lane.x != -1) {
            general_goal(current_pos.x,current_pos.y + 25);
            double lane_width = calculate_lane_width(left_lane, right_lane);
            // double left_lane_width = std::abs(left_lane.x - lane_hist.first.x);
            // double right_lane_width = std::abs(right_lane.y - lane_hist.second.y);
            // check_lane_consistency(current_pos, lane_width);
            
            if(lane_width > 1.4*8.0)// || lane_width > 1.1*8.0)
            {

                RCLCPP_WARN(get_logger(),"PARKING DETECTED Cond1: %d. Cond2 %d",lane_width > 1.2 * 8.0, lane_width > 1.1*8.0);
                visualize_lanes(left_lane, right_lane);
                
                RCLCPP_INFO(this->get_logger(),"Publishing Goal Info");
                if(left_found && !right_found)
                    pub_parking_goal(right_lane.x,left_lane.y + 20,yaw);
                else if(right_found && !left_found)
                    pub_parking_goal(left_lane.x,right_lane.y + 20,yaw);
                else
                {
                    RCLCPP_WARN(this->get_logger(),"Both the Ends Found");
                    if(left_x_hist.front() - left_x_hist.back() > 10)
                        pub_parking_goal(left_lane.x,right_lane.y + 20,yaw);
                    else
                        pub_parking_goal(left_lane.x,right_lane.y + 20,yaw);
                }
            }
            visualize_lanes(left_lane, right_lane);
        }

        if((left_lane.x == -1 && right_lane.x != -1))
        {
            // if(lane_width > 1.4*8.0)
            RCLCPP_WARN(get_logger(),"PARKING DETECTED Left");
            visualize_lanes(left_lane, right_lane);

        }
        else if(right_lane.x == -1 && left_lane.x != -1)
        {
            RCLCPP_WARN(get_logger(),"PARKING DETECTED Left");

        }
    }
    // Let's assume that the lanes are 8 meteres apart(worst case) and in that case worst case, the lanes can be 4 meters on
    // either side of the bot, so we start from the bot and go left and right 4 meters until we detect occupied lane cells.
    // Now in case of parking the offset will be greater than 2 meteres, so we can use that.
    std::pair<GridPoint, GridPoint> find_adjacent_lanes(const GridPoint& start, double yaw) {
        auto [fx, fy] = get_front_vector(yaw);
        auto [rx, ry] = get_front_vector(yaw + M_PI_2);  // Right perpendicular
    
        const int search_radius = static_cast<int>(8.0/current_map_->info.resolution);
        
        GridPoint left{-1,-1}, right{-1,-1};
        left_found = false;
        right_found = false;
        // Search in direction perpendicular to front vector
        // RCLCPP_WARN(this->get_logger(),"LE BHAI LANES");
        for (int i = 0; i <= search_radius; ++i) {
            GridPoint left_candidate{
                start.x + static_cast<int>(i * rx),
                start.y + static_cast<int>(i * ry)
            };
            
            GridPoint right_candidate{
                start.x - static_cast<int>(i * rx),
                start.y + static_cast<int>(i * ry)
            };
            if(left_candidate.y < start.y)
                left_candidate.y = start.y;
            if(right_candidate.y < start.y)
                right_candidate.y = start.y;

            if (is_lane_cell(left_candidate) && !left_found)
            {
                // RCLCPP_WARN(this->get_logger(),"Got left Lane Cell");
                left_found = true;
                left = left_candidate;
            }
            if (is_lane_cell(right_candidate) && !right_found)
            {
                // RCLCPP_WARN(this->get_logger(),"Got RIGHT Lane Cell");
                right_found = true;
                right = right_candidate;
            }
        }
        if(left_found && !right_found)
        {
            right.y = left.y;
            right.x = right_x_hist.back() + 50;
        }
        else if(right_found && !left_found)
        {
            left.y = right.y;
            left.x = left_x_hist.back() - 50;
        }
        else if(!right_found && !left_found)
        {
            RCLCPP_INFO(this->get_logger(),"Lane Free Zone Entered");
        }
        else
        {
            
            left_x_hist.push_back(left.x);
            right_x_hist.push_back(right.x);
        }
        
        return {left, right};
    }
    

    double calculate_lane_width(const GridPoint& left, const GridPoint& right) {
        double dx = (left.x - right.x) * current_map_->info.resolution;
        double dy = (left.y - right.y) * current_map_->info.resolution;
        return sqrt(dx*dx + dy*dy);
    }

    void check_lane_consistency(const GridPoint& pos, double yaw,double expected_width) {
        const int lookahead = 50;
        GridPoint ahead = get_lookahead_position(pos, lookahead);
        
        auto [future_left, future_right] = find_adjacent_lanes(ahead,yaw);
        if (future_left.x == -1 || future_right.x == -1) return;
        
        double future_width = calculate_lane_width(future_left, future_right);
        RCLCPP_INFO(get_logger(), "Lane width deviation detected: %.2f vs %.2f", 
        expected_width, future_width);
        if (abs(future_width - expected_width) > 0.2 * expected_width) {

            RCLCPP_WARN(get_logger(), "PARKING DETECTED");

        }
    }

    bool detect_parking_spot(const GridPoint& pos, double yaw) {
        auto [fx, fy] = get_front_vector(yaw);
        const int scan_depth = 10;  // 20 cells ahead
        const int scan_width = 100;  // 5 cells each side
    
        int free_count = 0;
        for (int d = 0; d < scan_depth; d++) {
            for (int w = -scan_width; w <= scan_width; w++) {
                GridPoint p{
                    pos.x + static_cast<int>(d*fx + w*fy),
                    pos.y + static_cast<int>(d*fy - w*fx)
                };
                if (is_parking_cell(p)) free_count++;
            }
        }
        return free_count > (scan_depth*scan_width*2*0.8);  // 80% free
    }
    
    void visualize_orientation(const GridPoint& pos, double yaw) {
        visualization_msgs::msg::Marker arrow;
        arrow.header.frame_id = "map";
        arrow.type = visualization_msgs::msg::Marker::ARROW;
        arrow.scale.x = 1.0;
        arrow.scale.y = 0.1;
        arrow.color.b = 1.0;
        arrow.color.a = 1.0;
    
        geometry_msgs::msg::Point start = grid_to_world(pos);
        arrow.points.push_back(start);
        
        geometry_msgs::msg::Point end;
        end.x = start.x + cos(yaw);
        end.y = start.y + sin(yaw);
        arrow.points.push_back(end);
        
        marker_pub_->publish(arrow);
    }
    
    void visualize_lanes(const GridPoint& left, const GridPoint& right) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.scale.x = 0.1;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        geometry_msgs::msg::Point left_p = grid_to_world(left);
        geometry_msgs::msg::Point right_p = grid_to_world(right);
        
        marker.points.push_back(left_p);
        marker.points.push_back(right_p);
        
        marker_pub_->publish(marker);
    }

    void visualize_parking_spot(const GridPoint& pos) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.scale.x = 2.0;
        marker.scale.y = 4.0;
        marker.scale.z = 0.1;
        marker.color.r = 1.0;
        marker.color.a = 0.5;

        geometry_msgs::msg::Point center = grid_to_world(pos);
        marker.pose.position = center;
        
        park_pub_->publish(marker);
    }

    // Helper functions
    GridPoint world_to_grid(double wx, double wy) {
        return {
            static_cast<int>((wx - current_map_->info.origin.position.x) / current_map_->info.resolution),
            static_cast<int>((wy - current_map_->info.origin.position.y) / current_map_->info.resolution)
        };
    }

    geometry_msgs::msg::Point grid_to_world(const GridPoint& p) {
        geometry_msgs::msg::Point wp;
        wp.x = p.x * current_map_->info.resolution + current_map_->info.origin.position.x;
        wp.y = p.y * current_map_->info.resolution + current_map_->info.origin.position.y;
        return wp;
    }

    bool is_lane_cell(const GridPoint& p) {
        return cell_value(p) > 50 && cell_value(p) <= 100;
    }

    bool is_parking_cell(const GridPoint& p) {
        return cell_value(p) == 0;
    }

    int8_t cell_value(const GridPoint& p) {
        if (p.x < 0 || p.x >= current_map_->info.width ||
            p.y < 0 || p.y >= current_map_->info.height) return -1;
            
        return current_map_->data[p.y * current_map_->info.width + p.x];
    }

    GridPoint get_lookahead_position(const GridPoint& pos, int cells) {
        // Simplified lookahead - implement actual orientation-based movement
        return {pos.x, pos.y+cells};
    }
    std::pair<GridPoint,GridPoint> lane_hist;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr park_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    std::vector<int> left_x_hist;
    std::vector<int> right_x_hist;
    bool left_found = false;
    bool right_found = false;
    double start_yaw = -100;
    nav_msgs::msg::OccupancyGrid::SharedPtr current_map_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaneAnalyzer>());
    rclcpp::shutdown();
    return 0;
}
