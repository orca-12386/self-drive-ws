// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/srv/lane_follow_toggle.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>
#include <cmath>
#include <vector>
#include <Eigen/Geometry>


// Using std::array<double, 2> for all points
using Point2D = std::array<double, 2>;

// Helper function to calculate the cross product magnitude of 2D vectors
double cross_product_magnitude(const Point2D& v1, const Point2D& v2) {
    // For 2D vectors, cross product magnitude is |v1×v2| = v1[0]*v2[1] - v1[1]*v2[0]
    return std::abs(v1[0] * v2[1] - v1[1] * v2[0]);
}

// Helper function to calculate dot product
double dot_product(const Point2D& v1, const Point2D& v2) {
    return v1[0] * v2[0] + v1[1] * v2[1];
}

// Helper function to subtract vectors
Point2D subtract_vectors(const Point2D& v1, const Point2D& v2) {
    return {v1[0] - v2[0], v1[1] - v2[1]};
}

// Helper function to check if vector is close to zero
bool is_vector_zero(const Point2D& v, double tolerance = 1e-10) {
    return (std::abs(v[0]) <= tolerance && std::abs(v[1]) <= tolerance);
}

double calculate_angle(const Point2D& candidate_goal, 
                      const Point2D& previous_goal, 
                      const Point2D& second_previous_goal) {
    // Convert goals to vectors for easier vector operations
    Point2D v1 = subtract_vectors(candidate_goal, previous_goal);
    Point2D v2 = subtract_vectors(second_previous_goal, previous_goal);
    
    // Check if any of the vectors have zero length
    if (is_vector_zero(v1) || is_vector_zero(v2)) {
        return 0.0;  // Return 0 degrees if any vector has zero length
    }
    
    // Calculate the angle between the two vectors using arctan2
    double angle = std::atan2(cross_product_magnitude(v1, v2), dot_product(v1, v2));
    
    // Convert angle from radians to degrees
    double angle_degrees = angle * 180.0 / M_PI;
    
    return angle_degrees;
}

// Assuming point is a 3D point and quaternion is a array with 4 elements
std::array<double, 3> get_point_at_distance(const std::array<double, 3>& point, 
    const std::array<double, 4>& quaternion, 
    double distance) {
// Create quaternion using Eigen
Eigen::Quaterniond quat(quaternion[3], quaternion[0], quaternion[1], quaternion[2]);
quat.normalize();

// Create forward vector [1, 0, 0]
Eigen::Vector3d forward_vector(1.0, 0.0, 0.0);

// Apply rotation to forward vector
Eigen::Vector3d rotated_vector = quat * forward_vector;

// Normalize rotated vector
rotated_vector.normalize();

// Scale by distance
rotated_vector *= distance;

// Add to point
std::array<double, 3> new_point = {
point[0] + rotated_vector[0],
point[1] + rotated_vector[1],
point[2] + rotated_vector[2]
};

return new_point;
}


/*
Get nearest point in both lanes
Hill climb using heuristic as goal angle
Midpoint of final points
def calculate_angle(candidate_goal, previous_goal, second_previous_goal):
    # Convert goals to numpy arrays for easier vector operations
    v1 = np.array(candidate_goal) - np.array(previous_goal)
    v2 = np.array(second_previous_goal) - np.array(previous_goal)
    # Check if any of the vectors have zero length
    if np.allclose(v1, 0) or np.allclose(v2, 0):
        return 0.0  # Return 0 degrees if any vector has zero length
    # Calculate the angle between the two vectors using arctan2
    angle = np.arctan2(np.linalg.norm(np.cross(v1, v2)), np.dot(v1, v2))
    # Convert angle from radians to degrees
    angle_degrees = np.degrees(angle)
    return angle_degrees

def calculate_goal_angle(self, goals):
    if len(goals)>=2:
        return calculate_angle(self.coords, NodeGlobal.goals[-1], NodeGlobal.goals[-2])
    else:
        return None
*/

double computeYaw(const std::array<double, 2>& start, const std::array<double, 2>& end) {
    int dx = end[0] - start[0];
    int dy = end[1] - start[1];
    return std::atan2(dy, dx); // returns yaw in radians
}

std::array<double, 2> convert_to_global_coords(std::array<int, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    double resolution = map->info.resolution;
    std::array<double, 2> origin = {map->info.origin.position.x, map->info.origin.position.y};
    std::array<double, 2> converted;
    for(int i = 0;i<2;i++) {
        converted[i] = origin[i] + (coords[i] * resolution);
    }
    return converted;
}

std::array<int, 2> convert_to_grid_coords(std::array<double, 2> coords, nav_msgs::msg::OccupancyGrid::SharedPtr map) {
    double resolution = map->info.resolution;
    std::array<double, 2> origin = {map->info.origin.position.x, map->info.origin.position.y};
    std::array<int, 2> converted;
    for(int i = 0;i<2;i++) {
        converted[i] = std::round((coords[i]-origin[i])/resolution);
    }
    return converted;
}

double calculate_distance(std::array<double, 2> p1, std::array<double, 2> p2) {
    return std::sqrt(std::pow(p1[0]-p2[0], 2) + std::pow(p1[1]-p2[1], 2));
}
double calculate_distance(std::array<int, 2> p1, std::array<int, 2> p2) {
    return std::sqrt(std::pow(p1[0]-p2[0], 2) + std::pow(p1[1]-p2[1], 2));
}


class LaneFollowerNode : public rclcpp::Node
{
public:
    LaneFollowerNode() : 
    rclcpp::Node("lane_follower_node")
    {
        RCLCPP_INFO(this->get_logger(), "lane_follower_node started");

        this->initialise_data();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&LaneFollowerNode::timer_callback, this));
    };

private:
    void initialise_data() {
        std::string map1_sub_topic("/map/white/local/near");
        std::string map2_sub_topic("/map/yellow/local/interp");

        map1_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map1_sub_topic, 10, std::bind(&LaneFollowerNode::map1Callback, this, std::placeholders::_1));
        
        map2_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map2_sub_topic, 10, std::bind(&LaneFollowerNode::map2Callback, this, std::placeholders::_1));
    
        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&LaneFollowerNode::odomCallback, this, std::placeholders::_1));

        goal_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

        map1_recv = false;
        map2_recv = false;
        odom_recv = false;
        
        goal_pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
        goal_pose_msg->header.frame_id = "odom";
        goal_pose_msg->header.stamp = this->now();
        goal_pose_msg->pose.position.z = 0.0;
        goal_pose_msg->pose.orientation.x = 0.0;
        goal_pose_msg->pose.orientation.y = 0.0;

        toggle_srv = this->create_service<interfaces::srv::LaneFollowToggle>("toggle_lane_follow", std::bind(&LaneFollowerNode::toggle_lane_follow, this, std::placeholders::_1, std::placeholders::_2));
        running = true;
        start = true;
    }

    void toggle_lane_follow(const std::shared_ptr<interfaces::srv::LaneFollowToggle::Request> request, 
        std::shared_ptr<interfaces::srv::LaneFollowToggle::Response> response) {
        RCLCPP_INFO(this->get_logger(), "toggling lane follow to: %s", request->toggle ? "ON" : "OFF");

        if (request->toggle) {
            // Turning ON
            if (!running) {
                // Cancel existing timer first to prevent multiple timers
                if (timer) {
                timer->cancel();
                timer.reset();  // Reset the shared_ptr
                }

                // Reset state
                this->start = true;
                this->goals.clear();
                this->orientations.clear();

                // Create new timer
                RCLCPP_INFO(this->get_logger(), "Creating new timer");
                timer = this->create_wall_timer(
                std::chrono::milliseconds(100), 
                std::bind(&LaneFollowerNode::timer_callback, this));

                this->running = true;
                RCLCPP_INFO(this->get_logger(), "Lane following started");
            } else {
                RCLCPP_INFO(this->get_logger(), "Lane following already running");
            }
        } else {
        // Turning OFF
            if (running) {
                if (timer) {
                    timer->cancel();
                    timer.reset();  // Reset the shared_ptr
                }
                this->running = false;
                RCLCPP_INFO(this->get_logger(), "Lane following stopped");
            } else {
                RCLCPP_INFO(this->get_logger(), "Lane following already stopped");
            }
        }

        response->success = true;
    }

    void map1Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map1_msg = msg;
        map1_recv = true;
    }

    void map2Callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map2_msg = msg;
        map2_recv = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        odom_recv = true;
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        orientations.push_back(yaw);
        if(orientations.size() > 5) {
            orientations.erase(orientations.begin());
        }
        double avg = 0;
        for(int i = 0;i<orientations.size();i++) {
            avg+=orientations[i];
        }
        avg/=orientations.size();
        average_orientation = avg;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

    double calculate_goal_angle(Point2D coords, const std::vector<Point2D>& goals) {
        return calculate_angle(coords, goals[goals.size()-1], goals[goals.size()-2]);
    }


    // OPTIMIZED: Radial search with early termination
    bool get_nearest_point_bfs(const std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::array<int, 2>& dst) {
        // Early exit if source is already occupied  
        const int src_index = src[1] * map->info.width + src[0];
        if(map->data[src_index] > 0) {
            dst = src;
            return true;
        }
        
        const int width = map->info.width;
        const int height = map->info.height;
        const int max_radius = static_cast<int>(std::ceil(3.0 / map->info.resolution));
        
        // Radial search: check all points at each radius level
        for(int radius = 1; radius <= max_radius && running; ++radius) {
            // Check points on the perimeter of the circle
            for(int dx = -radius; dx <= radius; ++dx) {
                for(int dy = -radius; dy <= radius; ++dy) {
                    // Skip points not on the current radius circle
                    if(dx*dx + dy*dy != radius*radius && 
                       !(std::abs(dx) == radius || std::abs(dy) == radius)) {
                        continue;
                    }
                    
                    const int nx = src[0] + dx;
                    const int ny = src[1] + dy;
                    
                    // Bounds check
                    if(nx < 0 || nx >= width || ny < 0 || ny >= height) {
                        continue;
                    }
                    
                    // Check occupancy
                    const int index = ny * width + nx;
                    if(map->data[index] > 0) {
                        dst = {nx, ny};
                        return true;
                    }
                }
            }
        }
        return false;
    }

    double get_nearest_point_distance(const std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        std::array<int,2> dst;
        bool status = get_nearest_point_bfs(src, map, dst);
        if(!status) {
            return 100;
        } else {
            return calculate_distance(src, dst);
        }
    }

    // OPTIMIZED: Sparse sampling with focused search
    std::array<int, 2> get_best_point_hill_climb(std::array<int, 2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        if(goals.size() < 2) {
            return src;
        }
        
        const double max_search_radius = 7.0;
        const double resolution = map->info.resolution;
        const int max_radius_grid = static_cast<int>(std::ceil(max_search_radius / resolution));
        const int width = map->info.width;
        const int height = map->info.height;
        
        std::array<int, 2> best_point = src;
        double best_angle = -1.0;
        
        // Pre-calculate map parameters for fast coordinate conversion
        const double origin_x = map->info.origin.position.x;
        const double origin_y = map->info.origin.position.y;
        
        // OPTIMIZATION: Use sparse sampling with multiple sampling densities
        const std::vector<int> sample_steps = {1, 2, 4}; // Different sampling densities
        
        for(int step : sample_steps) {
            for(int dx = -max_radius_grid; dx <= max_radius_grid; dx += step) {
                for(int dy = -max_radius_grid; dy <= max_radius_grid; dy += step) {
                    // Skip if outside circular search area
                    if(dx*dx + dy*dy > max_radius_grid*max_radius_grid) {
                        continue;
                    }
                    
                    const int nx = src[0] + dx;
                    const int ny = src[1] + dy;
                    
                    // Bounds check
                    if(nx < 0 || nx >= width || ny < 0 || ny >= height) {
                        continue;
                    }
                    
                    // Check occupancy
                    const int index = ny * width + nx;
                    if(map->data[index] > 0) {
                        // Fast coordinate conversion
                        const Point2D global_curr = {
                            origin_x + (nx * resolution),
                            origin_y + (ny * resolution)
                        };
                        
                        const double angle = calculate_goal_angle(global_curr, goals);
                        
                        if(best_angle < 0 || angle > best_angle) {
                            best_angle = angle;
                            best_point = {nx, ny};
                        }
                    }
                }
            }
            
            // If we found a good point with coarse sampling, refine around it
            if(best_angle > 0 && step > 1) {
                // Refine search around best point found so far
                const int refine_radius = step;  
                for(int dx = -refine_radius; dx <= refine_radius; ++dx) {
                    for(int dy = -refine_radius; dy <= refine_radius; ++dy) {
                        const int nx = best_point[0] + dx;
                        const int ny = best_point[1] + dy;
                        
                        if(nx < 0 || nx >= width || ny < 0 || ny >= height) {
                            continue;
                        }
                        
                        const int index = ny * width + nx;
                        if(map->data[index] > 0) {
                            const Point2D global_curr = {
                                origin_x + (nx * resolution),
                                origin_y + (ny * resolution)
                            };
                            
                            const double angle = calculate_goal_angle(global_curr, goals);
                            
                            if(angle > best_angle) {
                                best_angle = angle;
                                best_point = {nx, ny};
                            }
                        }
                    }
                }
            }
        }
        
        return best_point;
    }

    
    // Optimized distance calculation using squared distance when possible
    inline double calculate_distance_squared(const std::array<double, 2>& p1, const std::array<double, 2>& p2) {
        const double dx = p1[0] - p2[0];
        const double dy = p1[1] - p2[1];
        return dx*dx + dy*dy;
    }

    inline double calculate_distance_squared(const std::array<int, 2>& p1, const std::array<int, 2>& p2) {
        const double dx = p1[0] - p2[0];
        const double dy = p1[1] - p2[1];
        return dx*dx + dy*dy;
    }


    bool validate_goal(std::array<double, 2> goal) {
        bool angle_condition = calculate_goal_angle(goal, goals) > 100;
        std::array<int, 2> goal_grid = convert_to_grid_coords(goal, map1_msg);
        bool obstacle1_condition = map1_msg->data[goal_grid[1]*map1_msg->info.width + goal_grid[0]] <= 0;
        bool obstacle2_condition = map2_msg->data[goal_grid[1]*map2_msg->info.width + goal_grid[0]] <= 0;
        bool lane1_clearance_condition = get_nearest_point_distance(goal_grid, map1_msg) > 0.4;
        bool lane2_clearance_condition = get_nearest_point_distance(goal_grid, map2_msg) > 0.4;
        bool not_previous_goal = true;
        if(!this->start) {
            not_previous_goal = calculate_distance(goal, prev_goal) > 0.1;
        }
        bool max_bot_dist = calculate_distance(goal, {odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y}) < 7;
        return angle_condition && obstacle1_condition && obstacle2_condition && not_previous_goal && max_bot_dist && lane1_clearance_condition && lane2_clearance_condition;
    }

    void publish_goal(const nav_msgs::msg::Odometry::SharedPtr odom, const nav_msgs::msg::OccupancyGrid::SharedPtr map1, const nav_msgs::msg::OccupancyGrid::SharedPtr map2) {
        if(!running) {
            return;
        }
        std::array<double, 2> bot_position = {odom->pose.pose.position.x, odom->pose.pose.position.y};
        std::array<int, 2> bot_position_grid = convert_to_grid_coords(bot_position, map1);
        if(this->start) {
            std::array<double, 3> bot_position_3d = {bot_position[0], bot_position[1], odom->pose.pose.position.z};
            exp_average_goal_orientation = average_orientation;
            std::array<double, 4> quaternion = {odom->pose.pose.orientation.x,
                odom->pose.pose.orientation.y,
                odom->pose.pose.orientation.z,
                odom->pose.pose.orientation.w
            };
            std::array<double, 3> ext = get_point_at_distance(bot_position_3d, quaternion, 1.5);
            std::array<double, 2> ext2d = {ext[0], ext[1]};
            goals.push_back(bot_position);
            goals.push_back(ext2d);
            this->start = false;
        }
        std::array<int, 2> map1_src, map2_src;
        bool status1 = get_nearest_point_bfs(bot_position_grid, map1, map1_src);
        bool status2 = get_nearest_point_bfs(bot_position_grid, map2, map2_src);
        if(!(status1 && status2)) {
            log("Could not find nearest points");
            return;
        }
        std::array<int, 2> parent1_grid, parent2_grid;
        parent1_grid = get_best_point_hill_climb(map1_src, map1);
        parent2_grid = get_best_point_hill_climb(map2_src, map2);
        std::array<double, 2> parent1, parent2;
        parent1 = convert_to_global_coords(parent1_grid, map1);
        parent2 = convert_to_global_coords(parent2_grid, map2);
        std::array<double, 2> goal = {(parent1[0]+parent2[0])/2, (parent1[1]+parent2[1])/2};
        // log(std::to_string(bot_position[0]) + std::string(", ") + std::to_string(bot_position[1]));
        // log(std::to_string(map1_src[0]) + std::string(", ") + std::to_string(map1_src[1]));
        // log(std::to_string(map2_src[0]) + std::string(", ") + std::to_string(map2_src[1]));
        // log(std::to_string(parent1[0]) + std::string(", ") + std::to_string(parent1[1]));
        // log(std::to_string(parent2[0]) + std::string(", ") + std::to_string(parent2[1]));
        // log(std::to_string(goal[0]) + std::string(", ") + std::to_string(goal[1]));
        if(!running) {
            return;
        }

        if(validate_goal(goal)) {
            double goal_orientation = computeYaw(bot_position, goal);
            double alpha = 0.7;
            exp_average_goal_orientation = (alpha*exp_average_goal_orientation)+((1-alpha)*goal_orientation);    
            goal_pose_msg->pose.position.x = goal[0];
            goal_pose_msg->pose.position.y = goal[1];
            goal_pose_msg->pose.orientation.z = sin(exp_average_goal_orientation / 2);
            goal_pose_msg->pose.orientation.w = cos(exp_average_goal_orientation / 2);
            goal_pub->publish(*goal_pose_msg);
            prev_goal = goal;
            if(goals.size() <= 1) {
                goals.push_back(goal);
            } else if(goals.size() > 0) {
                if(calculate_distance(goal, goals[goals.size()-1]) >= 2) {
                    goals.push_back(goal);
                }    
            }
        } else {
            goal_pose_msg->pose.position.x = goals[goals.size()-1][0];
            goal_pose_msg->pose.position.y = goals[goals.size()-1][1];
            goal_pose_msg->pose.orientation.z = sin(exp_average_goal_orientation / 2);
            goal_pose_msg->pose.orientation.w = cos(exp_average_goal_orientation / 2);
            goal_pub->publish(*goal_pose_msg);
        }
        if(goals.size() > 10) {
            goals.erase(goals.begin());
        }
    }

    void timer_callback() {
        if(!running) {
            return;
        }
        if(map1_recv && map2_recv && odom_recv) {
            publish_goal(odometry_msg, map1_msg, map2_msg);
        } else {
            static int counter = 0;
            if (counter % 50 == 0) {
                RCLCPP_INFO(this->get_logger(), "Waiting for subscriptions - map1: %s, map2: %s, odom: %s", 
                            map1_recv ? "OK" : "NO", 
                            map2_recv ? "OK" : "NO", 
                            odom_recv ? "OK" : "NO");
                counter = 0;
            }
            counter++;
        }
    }

    rclcpp::Service<interfaces::srv::LaneFollowToggle>::SharedPtr toggle_srv;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map1_sub, map2_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    bool map1_recv, map2_recv, odom_recv;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr map1_msg, map2_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;
    geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_msg;
    rclcpp::TimerBase::SharedPtr timer;
    std::vector<double> orientations;
    std::vector<Point2D> goals;
    std::array<double, 2> prev_goal;
    bool running;
    bool start;
    double average_orientation;
    double exp_average_goal_orientation;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneFollowerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}