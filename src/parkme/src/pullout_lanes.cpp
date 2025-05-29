#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "interfaces/action/goal_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "utils.cpp"

namespace parkme
{
class ParkingActionServer : public rclcpp::Node
{
public:
    using Parking = interfaces::action::GoalAction;
    using GoalHandleParking = rclcpp_action::ServerGoalHandle<Parking>;
    explicit ParkingActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("PullOut",options)
    {
        this->declare_parameter<double>("distance_ahead", 100.0);// Distance infront of the bot where the goals will be given
        this->declare_parameter<double>("distance_angular", 50.0);// Once it has pulled out, how far left or right will the next goal be to make it
        //face the drum, set this irl depending upon how far has the drum been placed from the pullout spot.
        this->declare_parameter<double>("drum_dist_thresh", 1.0); //Distance from the drum where the bot must stop approx 3 ft =  1 meter
        this->declare_parameter<std::string>("pullout_mode", "right");
        
        distance_ahead = this->get_parameter("distance_ahead").as_double(); //How far ahead should the goal be published, for the final goal, this decides
                                                                            //how far from the detected marker will the goal be published

        distance_angular = this->get_parameter("distance_angular").as_double(); //How far left or right, left or right is automatically calculated based
                                                                                //on angle

        pullout_mode = this->get_parameter("pullout_mode").as_string();
        
        drum_dist_thresh = this->get_parameter("drum_dist_thresh").as_double();

        if(pullout_mode == "right")
            distance_angular = -distance_angular;

        white_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/white/local", 10, std::bind(&ParkingActionServer::white_map_callback, this, std::placeholders::_1));
        
        yellow_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/map/yellow/local", 10, std::bind(&ParkingActionServer::yellow_map_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&ParkingActionServer::odom_callback, this, std::placeholders::_1));
        drum_dist_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/detector/traffic_drum/coordinates",10,std::bind(&ParkingActionServer::drum_dist_callback,this,std::placeholders::_1));


        near_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/near_lane_point", 10);
        far_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("/far_lane_point", 10);
        goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        using namespace std::placeholders;

        this->action_server_ = rclcpp_action::create_server<Parking>(
          this,
          "parking",
          std::bind(&ParkingActionServer::handle_goal, this, _1, _2),
          std::bind(&ParkingActionServer::handle_cancel, this, _1),
          std::bind(&ParkingActionServer::handle_accepted, this, _1));
    }

private:
    bool pullout_request = false, map_rec = false, moving_to_drum = false, pulled_out = false,drum_dist_logged = false;
    double theta = 0.0, log_angle = 0.0,dist_drum = 10.0, drum_dist_thresh = 1.0, distance_ahead = 50.0,distance_angular = 50.0, angled_goal_yaw = 0.0;
    std::string pullout_mode = "left";


    nav_msgs::msg::OccupancyGrid::SharedPtr current_white_grid_, current_yellow_grid_;
    geometry_msgs::msg::Pose latest_pose_;

    rclcpp_action::Server<Parking>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr white_map_sub_, yellow_map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr drum_dist_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr near_marker_pub_,far_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;


    BotPose current_pose;
    Map current_white_map, current_yellow_map;
    MapPose nearest_lane_mp, farthest_lane_mp, goal_pose, angled_goal_pose;
    WorldPose nearest_lane_wp, farthest_lane_wp, angled_goal_wp;

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
    
    void drum_dist_callback(const geometry_msgs::msg::Point coords)
    {
        drum_dist_logged = true;
        dist_drum = coords.z;
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
        pullout_request = true;
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
        goal_pose.x = current_pose.map_pose.x;
        goal_pose.y = current_pose.map_pose.y;
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


    void white_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_white_grid_ = msg;
        current_white_map.height     = msg->info.height;
        current_white_map.width      = msg->info.width;
        current_white_map.resolution = msg->info.resolution;
        current_white_map.origin.x   = msg->info.origin.position.x;
        current_white_map.origin.y   = msg->info.origin.position.y;
    
        current_white_map.grid.resize(
            current_white_map.height, std::vector<int>(current_white_map.width, -1)
        );
    
        for (int y = 0; y < current_white_map.height; y++) {
          for (int x = 0; x < current_white_map.width; x++) {
            int index = y * current_white_map.width + x;
            current_white_map.grid[y][x] = msg->data[index];
          }
        }
        // RCLCPP_INFO(this->get_logger(),"Got Grid");
        map_rec = true;
    }

    void yellow_map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        current_yellow_grid_ = msg;
        current_yellow_map.height     = msg->info.height;
        current_yellow_map.width      = msg->info.width;
        current_yellow_map.resolution = msg->info.resolution;
        current_yellow_map.origin.x   = msg->info.origin.position.x;
        current_yellow_map.origin.y   = msg->info.origin.position.y;
    
        current_yellow_map.grid.resize(
            current_yellow_map.height, std::vector<int>(current_yellow_map.width, -1)
        );
    
        for (int y = 0; y < current_yellow_map.height; y++) {
          for (int x = 0; x < current_yellow_map.width; x++) {
            int index = y * current_yellow_map.width + x;
            current_yellow_map.grid[y][x] = msg->data[index];
          }
        }
        // RCLCPP_INFO(this->get_logger(),"Got Grid");
        map_rec = true;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if(!map_rec)
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for Occupancy Grid");
            return;
        }
        if(!pullout_request)
        {
            RCLCPP_INFO(this->get_logger(),"Waiting for Client");
            return;
        }

    
        current_pose.world_pose.x = msg->pose.pose.position.x;
        current_pose.world_pose.y = msg->pose.pose.position.y;
        current_pose.map_pose = Utils::getMapPoseFromWorldPose(current_pose.world_pose,current_white_map);

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );
        tf2::Matrix3x3(q).getRPY(current_pose.roll, current_pose.pitch, current_pose.yaw);


        nearest_lane_mp = Utils::findClosestForValue(current_pose.map_pose,current_white_map,200,100);
        nearest_lane_wp = Utils::getWorldPoseFromMapPose(nearest_lane_mp,current_white_map);

        log_angle = Utils::getAngleRadians(current_pose.world_pose,nearest_lane_wp);
        RCLCPP_INFO(this->get_logger(),"Angle: %f",log_angle);
        
        if(std::abs(log_angle) < 0.5 && !moving_to_drum)
        {
            if(!pulled_out)
            {
                pulled_out = true;
                angled_goal_pose = get_goal_infront(msg,-30.0,distance_angular,&nearest_lane_mp);
                angled_goal_wp = Utils::getWorldPoseFromMapPose(angled_goal_pose,current_white_map);

                if(pullout_mode == "right") //if mode is right turn pullout, the goal needs to be oriented right
                    theta = theta - M_PI_2;
                else                       // Left Orientation for left turn pullout
                    theta = theta + M_PI_2;
                
                angled_goal_yaw = theta;
                general_goal(angled_goal_pose,theta);
                RCLCPP_INFO(this->get_logger(),"Pulled Out! Making the bot face the Drum with goal%d,%d",goal_pose.x,goal_pose.y);
            }
            else
            {
                double dist_to_last_goal = Utils::worldDistance(current_pose.world_pose,angled_goal_wp);
                if(dist_to_last_goal < 0.5 && std::abs(current_pose.yaw - angled_goal_yaw) < 0.3)
                    moving_to_drum = true;
                WorldPose gp = angled_goal_wp;
                RCLCPP_INFO(this->get_logger(),"Bot: %f %f  Goal: %f %f",current_pose.world_pose.x, current_pose.world_pose.y,gp.x,gp.y);
                RCLCPP_INFO(this->get_logger(),"Checking if last goal %d, %d reached, straight goals will begin after %f",angled_goal_pose.x,angled_goal_pose.y,dist_to_last_goal);
            }                
        }
        else if(!pulled_out)
        {
                goal_pose = get_goal_infront(msg,distance_ahead,0.0);
                general_goal(goal_pose,theta);
        }
        else if(moving_to_drum)
        {
            if(dist_drum > drum_dist_thresh)
            {
                if(drum_dist_logged)
                {
                    distance_ahead = (dist_drum - 0.5)/current_white_map.resolution;
                    RCLCPP_INFO(this->get_logger(),"Distance from Barrel: %f",dist_drum);
                }
                goal_pose = get_goal_infront(msg,distance_ahead,0.0);
                general_goal(goal_pose,theta);
            }
            else
            {
                // general_goal(current_pose.map_pose,theta);
                // std::system("ros2 lifecycle set /controller_server shutdown");
                RCLCPP_INFO(this->get_logger(),"Objective Complete");
                rclcpp::shutdown();
            }
        }
        publishMarker(nearest_lane_wp.x, nearest_lane_wp.y, 1);
        if(!pulled_out)
            RCLCPP_INFO(this->get_logger(),"Pulling Out");

    }
    void general_goal(MapPose pose,double theta)
    {
        WorldPose wp = Utils::getWorldPoseFromMapPose(pose,current_white_map); 

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
    
    };
}
    RCLCPP_COMPONENTS_REGISTER_NODE(parkme::ParkingActionServer)