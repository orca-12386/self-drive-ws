#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>          // For quaternion operations
#include <tf2/LinearMath/Matrix3x3.h>           // For converting quaternion to roll, pitch, yaw
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For geometry_msgs compatibility with tf2
#include <tf2_ros/buffer.h>                     // For transform buffer
#include <tf2_ros/transform_listener.h> 
#include <queue>
#include <vector>
#include <cmath>

#include "utils.cpp"


#include "interfaces/action/goal_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace parkme
{
class ParkingActionServer : public rclcpp::Node {
    public:
        using Parking = interfaces::action::GoalAction;
        using GoalHandleParking = rclcpp_action::ServerGoalHandle<Parking>;
        explicit ParkingActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("parking_action_server", options)
        {
            this->declare_parameter<double>("distance_ahead", 100.0);
            this->declare_parameter<double>("distance_angular", 50.0);
            this->declare_parameter<std::string>("parking_mode", "pullin");
            
            distance_ahead = this->get_parameter("distance_ahead").as_double(); //How far ahead should the goal be published, for the final goal, this decides
                                                                                //how far from the detected marker will the goal be published

            distance_angular = this->get_parameter("distance_angular").as_double(); //How far left or right, left or right is automatically calculated based
                                                                                    //on angle
            parking_mode = this->get_parameter("parking_mode").as_string(); //mode decides the orientation of the final goal


            grid_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/map/white/local", 10, std::bind(&ParkingActionServer::grid_callback, this, std::placeholders::_1));
            
            pose_sub_ = create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&ParkingActionServer::pose_callback, this, std::placeholders::_1));
            
            near_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/near_lane_point", 10);
            far_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/far_lane_point", 10);
            goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
            RCLCPP_INFO(this->get_logger(),"Parking Node Initialized");
            using namespace std::placeholders;

            this->action_server_ = rclcpp_action::create_server<Parking>(
              this,
              "parking",
              std::bind(&ParkingActionServer::handle_goal, this, _1, _2),
              std::bind(&ParkingActionServer::handle_cancel, this, _1),
              std::bind(&ParkingActionServer::handle_accepted, this, _1));

            
        }
    
    private:
        struct GridCell {
            int x, y;
        };
        rclcpp_action::Server<Parking>::SharedPtr action_server_;
        std::vector<std::vector<bool>> visited_;
        nav_msgs::msg::OccupancyGrid::SharedPtr current_grid_;
        
        double distance_ahead = 0.0,distance_angular = 0.0,log_angle = 0.0,theta = 0.0;
        std::string parking_mode;
        Map current_map_white;
        BotPose current_pose_, prev_pose;
        MapPose farthest_white_mp,nearest_white_mp,final_goal_pose;

        bool got_odom,got_white_map,park_request = false,parked = false;
        std::shared_ptr<GoalHandleParking> goal_handler;
        
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID & uuid,
            std::shared_ptr<const Parking::Goal> goal)
          {
            RCLCPP_INFO(this->get_logger(), "Received goal request with command:%d", goal->data);
            (void)uuid;
            
            if(goal->data == 1)
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            return rclcpp_action::GoalResponse::REJECT;
          }
        
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandleParking> goal_handle)
          {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            (void)goal_handle;
            return rclcpp_action::CancelResponse::ACCEPT;
          }
        
        void handle_accepted(const std::shared_ptr<GoalHandleParking> goal_handle)
          {
            using namespace std::placeholders;
            // this needs to return quickly to avoid blocking the executor, so spin up a new thread
            std::thread{std::bind(&ParkingActionServer::execute, this, _1), goal_handle}.detach();
          }
        

        void execute(const std::shared_ptr<GoalHandleParking> goal_handle)
          {
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate loop_rate(1);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Parking::Feedback>();
            feedback->feedback = 1;
            auto & action_status = feedback->feedback;
            auto result = std::make_shared<Parking::Result>();    

            park_request = true;
            goal_handler = goal_handle;

          }
        
        void grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            current_map_white.height     = msg->info.height;
            current_map_white.width      = msg->info.width;
            current_map_white.resolution = msg->info.resolution;
            current_map_white.origin.x   = msg->info.origin.position.x;
            current_map_white.origin.y   = msg->info.origin.position.y;
        
            current_map_white.grid.resize(
                current_map_white.height, std::vector<int>(current_map_white.width, -1)
            );
        
            for (int y = 0; y < current_map_white.height; y++) {
              for (int x = 0; x < current_map_white.width; x++) {
                int index = y * current_map_white.width + x;
                current_map_white.grid[y][x] = msg->data[index];
              }
            }
        
            current_grid_ = msg;
            got_white_map = true;
            // RCLCPP_INFO(this->get_logger(),"Got Grid");
    
        }
    
        MapPose get_goal_infront(const nav_msgs::msg::Odometry::SharedPtr msg,double distance_ahead = 20.0,double distance_angular = 0,MapPose* marker=nullptr)
        {       
            // Extract orientation as yaw
            tf2::Quaternion q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
        
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            
            MapPose goal_pose;
            if(!marker)
            {
            goal_pose.x = current_pose_.map_pose.x;
            goal_pose.y = current_pose_.map_pose.y;
            }
            else
            {
                goal_pose.x = marker->x;
                goal_pose.y = marker->y;
            }
            // Determine dominant axis
            if (std::abs(std::cos(yaw)) > std::abs(std::sin(yaw))) {
                //Facing mostly towards X
                if(std::cos(yaw) > 0)
                {
                    goal_pose.x += distance_ahead * 1;
                    goal_pose.y += distance_angular * 1;
                    theta = 0;
                }else{
                    goal_pose.x += distance_ahead * -1;
                    goal_pose.y += distance_angular * -1;
                    theta = M_PI;
                }
            } else {
                //Facing mostly towards X
                if(std::sin(yaw) > 0)
                {
                    goal_pose.y += distance_ahead * 1;
                    goal_pose.x += distance_angular * 1;
                    theta = M_PI_2;
                }
                else
                {
                    goal_pose.y += distance_ahead * -1;
                    goal_pose.x += distance_angular * -1;
                    theta = -M_PI_2;
                }    
            }
            return goal_pose;
        }
        
        void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            got_odom = true;
            current_pose_.world_pose.x = msg->pose.pose.position.x;
            current_pose_.world_pose.y = msg->pose.pose.position.y;
            
            if(!got_white_map)
            {
                RCLCPP_INFO(this->get_logger(),"Waiting for Occupancy Grid");
                return;
            }
            if(!park_request)
            {
                RCLCPP_INFO(this->get_logger(),"Waiting for Client");
                return;
            }
                
            if(log_angle > 2.0)
            {
                RCLCPP_INFO(this->get_logger(),"Published Final goal near marker");
                WorldPose farthest_white_wp = Utils::getWorldPoseFromMapPose(farthest_white_mp,current_map_white);

                double angle = Utils::getAngleRadians(farthest_white_wp,current_pose_.world_pose);
                double distance_angular = 0.0;
                
                //This decides whether the parking spot is on the right or the left, if angle is positive, 
                //the goal is on the right and hence the goal is shifted a bit to the right by the distance_angular
                //parameter, and vice versa for negative angle or left side.
                if(angle > 0)
                    distance_angular = distance_angular;
                else
                    distance_angular = -distance_angular;   
                
                if(!parked)
                    final_goal_pose = get_goal_infront(msg,100,distance_angular,&nearest_white_mp);
                
                parked = true;
                WorldPose goal_pose_wp = Utils::getWorldPoseFromMapPose(final_goal_pose,current_map_white);

                if(parking_mode == "parallel")
                    theta += M_PI_2;

                general_goal(final_goal_pose,theta = theta);
    
                double goal_dist = Utils::worldDistance(current_pose_.world_pose,goal_pose_wp);
                
                if(goal_dist < 0.2)
                {
                    parked = true;
                    auto result = std::make_shared<Parking::Result>();
                    result->success = true;
                    goal_handler->succeed(result);
                    RCLCPP_INFO(this->get_logger(), "Parking Complete");
                    rclcpp::shutdown();
                    return;
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(),"Published Normal Goal with angle %f",log_angle);
                current_pose_.map_pose = Utils::getMapPoseFromWorldPose(current_pose_.world_pose,current_map_white);
    
                MapPose goal_pose = get_goal_infront(msg,distance_ahead);
                general_goal(goal_pose,theta = theta);
    
                if (Utils::worldDistance(current_pose_.world_pose, prev_pose.world_pose) <
                    0.1) {
                theta = current_pose_.yaw;
                } else {
                theta =
                    Utils::getAngleRadians(prev_pose.world_pose, current_pose_.world_pose);
                }
            
                Utils::removeMapBehindBot(
                    current_map_white,current_pose_.world_pose, theta, 20, 20
                );
            
    
                if (!got_odom ||
                    Utils::worldDistance(prev_pose.world_pose, current_pose_.world_pose) > 3) {
                    prev_pose = current_pose_;
                }
    
                if (got_white_map) {
                current_pose_.map_pose =
                    Utils::getMapPoseFromWorldPose(current_pose_.world_pose, current_map_white);
                }
    
            
                nearest_white_mp = Utils::findClosestForValue(
                    current_pose_.map_pose, current_map_white, 100, 100
                );
            
                farthest_white_mp =
                    Utils::exploreLane(nearest_white_mp, current_map_white);
    
                WorldPose nearest_white_wp = Utils::getWorldPoseFromMapPose(nearest_white_mp,current_map_white);
                WorldPose farthest_white_wp = Utils::getWorldPoseFromMapPose(farthest_white_mp,current_map_white);
                RCLCPP_INFO(this->get_logger(),"Near Point: %f %f",nearest_white_wp.x,nearest_white_wp.y);
                RCLCPP_INFO(this->get_logger(),"Far Point: %f %f",farthest_white_wp.x,farthest_white_wp.y);
                double dy = farthest_white_wp.y - nearest_white_wp.y;
                double dx = farthest_white_wp.x - nearest_white_wp.x;
    
                log_angle = std::atan2(dy,dx);
        
                            
                RCLCPP_INFO(this->get_logger(),"Angle %f",log_angle);
                publishMarker(nearest_white_wp.x, nearest_white_wp.y, 1);
                publishMarker(farthest_white_wp.x, farthest_white_wp.y, 1,1.0);
            }
            // RCLCPP_INFO(this->get_logger(),"Got Pose");
        }
    
        void general_goal(MapPose pose,double theta)
        {
            WorldPose wp = Utils::getWorldPoseFromMapPose(pose,current_map_white); 
    
            geometry_msgs::msg::PoseStamped goal;
            goal.pose.position.x = wp.x;
            goal.pose.position.y = wp.y;
            goal.header.frame_id = "map";
            tf2::Quaternion quat;
    
            quat.setRPY(0,0,theta);
    
            goal.pose.orientation.x = quat.x();
            goal.pose.orientation.y = quat.y();
            goal.pose.orientation.z = quat.z();
            goal.pose.orientation.w = quat.w();
            goal_pub_->publish(goal);
        }
    
        void publishMarker(double x, double y, int id, double green = 0.0) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp    = this->now();
            marker.ns              = "Parking Markers";
            marker.id              = id;
            marker.type            = visualization_msgs::msg::Marker::SPHERE;
            marker.action          = visualization_msgs::msg::Marker::ADD;
        
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0.0;
        
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
        
            marker.scale.x = 1.0;
            marker.scale.y = 1.0;
            marker.scale.z = 1.0;
        
            marker.color.r = 0.0;
            marker.color.g = green;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
        
            marker.lifetime = rclcpp::Duration(0, 0);
            if(green)
                far_marker_pub_->publish(marker);
            else
                near_marker_pub_->publish(marker);
        }
    
        // Helper functions
        bool is_obstacle(GridCell cell) {
            int index = cell.x + cell.y * current_grid_->info.width;
            return current_grid_->data[index] > 65;
        }
    
        GridCell world_to_grid(geometry_msgs::msg::PoseWithCovariance position){
            int x = (position.pose.position.x - current_grid_->info.origin.position.x) / current_grid_->info.resolution;
            int y = (position.pose.position.y - current_grid_->info.origin.position.y) / current_grid_->info.resolution;
            return {x, y};
        }
        // ROS members
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr near_marker_pub_,far_marker_pub_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
        
        // Constants
        std::vector<std::vector<int>> no_x_directions = {
            // {-1,  0},  // up
            // { 1,  0},  // down
            { 0, -1},  // left
            { 0,  1},  // right
            {-1, -1},  // up-left
            {-1,  1},  // up-right
            { 1, -1},  // down-left
            { 1,  1}   // down-right
        };
    
        std::vector<std::vector<int>> no_y_directions = {
            {-1,  0},  // up
            { 1,  0},  // down
            // { 0, -1},  // left
            // { 0,  1},  // right
            {-1, -1},  // up-left
            {-1,  1},  // up-right
            { 1, -1},  // down-left
            { 1,  1}   // down-right
        };
    
        std::vector<std::vector<bool>> create_visited_matrix() {
            return std::vector<std::vector<bool>>(
                current_grid_->info.width, 
                std::vector<bool>(current_grid_->info.height, false));
        }
    
        bool is_valid(GridCell cell) {
            return cell.x >= 0 && cell.x < (int)current_grid_->info.width &&
                   cell.y >= 0 && cell.y < (int)current_grid_->info.height;
        }
    };
    
    // int main(int argc, char** argv) {
    //     rclcpp::init(argc, argv);
    //     std::cout<<"Parking Node Initialized"<<std::endl;
    //     rclcpp::spin(std::make_shared<ParkingActionServer>());
    //     rclcpp::shutdown();
    //     return 0;
    // }
}
    RCLCPP_COMPONENTS_REGISTER_NODE(parkme::ParkingActionServer)
    