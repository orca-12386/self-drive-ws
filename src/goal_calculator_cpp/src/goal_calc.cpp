#include <rclcpp/rclcpp.hpp>
#include <map>
#include <tuple>
#include <cmath>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include <vector>
#include <string>
#include <optional>
#include <chrono>
#include <numeric>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <unordered_set>
#include <deque>
#include <Eigen/Dense>
#include <nanoflann.hpp>
#include <memory>
#include <iostream>
#include <iomanip>


using namespace std;

class Point;

template <class T>
class Message;

template <class messageDatatype, class parsedDatatype>
class Subscription;

template <class messageDatatype>
class Publishing;

class CandidateGoal;
class GoalCalculator;


class Config {
    public:
        static YAML::Node config;
        static vector<Point> goals;
        static rclcpp::Node* nodeptr;
        Config() {
            // std::string config_path = ament_index_cpp::get_package_share_directory("goal_calculator_cpp") + "/config/config.yaml";
            std::string config_path = "/mnt/d/ros2-igvc-autonav-sim/src/goal_calculator_cpp/config/config.yaml";
            config = YAML::LoadFile(config_path);
        }
        static void write_info(const std::string& log) {
            RCLCPP_INFO(nodeptr->get_logger(), "%s", log.c_str());
        }
};
YAML::Node Config::config;
vector<Point> Config::goals;
rclcpp::Node* Config::nodeptr;

class Point {
    public:
        double x;
        double y;

        Point(double x, double y) {
            this->x = x;
            this->y = y;
        };

        Point(double coord[2]) {
            this->x = coord[0];
            this->y = coord[1];
        }

        Point(vector<double> coord) {
            this->x = coord.begin()[0];
            this->y = coord.begin()[1];
        }

        double calculate_distance(Point p) {
            return sqrt(pow((this->x - p.x), 2) + pow(this->y-p.y, 2));
        };

        double calculate_angle(Point p1, Point p2) {
            //returns angle at p1 of angle this-p1-p2
            double BAx = this->x - p1.x;
            double BAy = this->y - p1.y;
            double BCx = p2.x - p1.x;
            double BCy = p2.y - p1.y;
            double dotProduct = BAx * BCx + BAy * BCy;
            double magnitudeBA = sqrt(BAx * BAx + BAy * BAy);
            double magnitudeBC = sqrt(BCx * BCx + BCy * BCy);
            double angleRad = acos(dotProduct / (magnitudeBA * magnitudeBC));
            double angleDeg = angleRad * 180.0 / M_PI;
            return angleDeg;
        }

        Point convert_to_global_coords(nav_msgs::msg::OccupancyGrid map_message) {
            Point origin = Point((double) map_message.info.origin.position.x, (double) map_message.info.origin.position.y);
            Point newcoord = Point(origin.x+(map_message.info.resolution*this->x), origin.y+(map_message.info.resolution*this->y));
            return newcoord;
        }

        Point convert_to_grid_coords(nav_msgs::msg::OccupancyGrid map_message) {
            Point origin = Point((double) map_message.info.origin.position.x, (double) map_message.info.origin.position.y);
            Point newcoord = Point(-origin.x+(this->x/map_message.info.resolution), -origin.y+(this->y/map_message.info.resolution));
            return newcoord;
        }
};

vector<vector<Point>> dbscan( vector<vector<int>>& grid, double eps, int minPts) {
    int rows = grid.size();
    int cols = grid[0].size();
    vector<vector<bool>> visited(rows, vector<bool>(cols, false));
    vector<vector<Point>> clusters;

    auto is_valid = [&](int x, int y) {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    };

    auto get_neighbors = [&](int x, int y) {
        vector<Point> neighbors;
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                if (dx == 0 && dy == 0) continue;
                int nx = x + dx, ny = y + dy;
                if (is_valid(nx, ny) && grid[nx][ny] == 1) {
                    neighbors.emplace_back(nx, ny);
                }
            }
        }
        return neighbors;
    };

    function<void(int, int, vector<Point>&)> expand_cluster = [&](int x, int y, vector<Point>& cluster) {
        cluster.emplace_back(x, y);
        visited[x][y] = true;

        vector<Point> neighbors = get_neighbors(x, y);
        if (neighbors.size() >= minPts) {
            for ( auto& neighbor : neighbors) {
                if (!visited[neighbor.x][neighbor.y]) {
                    expand_cluster(neighbor.x, neighbor.y, cluster);
                }
            }
        }
    };

    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            if (!visited[i][j] && grid[i][j] == 1) {
                vector<Point> cluster;
                expand_cluster(i, j, cluster);
                if (cluster.size() >= minPts) {
                    clusters.push_back(cluster);
                }
            }
        }
    }

    // Sort clusters by size in descending order
    sort(clusters.begin(), clusters.end(), []( vector<Point>& a,  vector<Point>& b) {
        return a.size() > b.size();
    });

    // Return the top 2 largest clusters
    return vector<vector<Point>>(clusters.begin(), clusters.begin() + min(2, static_cast<int>(clusters.size())));
};


struct PointPair {
    Point p1;
    Point p2;
    double distance;
    
    PointPair(Point p1, Point p2, double distance) : p1(p1), p2(p2), distance(distance) {}
};

vector<PointPair> findNearestPointPairs( vector<Point>& vec1,  vector<Point>& vec2, double tolerance = 0.1) {
    vector<PointPair> result;

    auto isMidpointUnique = [&result, tolerance]( Point& midpoint) {
        for ( auto& pair : result) {
            Point existingMidpoint((pair.p1.x + pair.p2.x) / 2, (pair.p1.y + pair.p2.y) / 2);
            if (std::abs(existingMidpoint.x - midpoint.x) < tolerance &&
                std::abs(existingMidpoint.y - midpoint.y) < tolerance) {
                return false;
            }
        }
        return true;
    };

    for ( auto& p1 : vec1) {
        double minDist = std::numeric_limits<double>::max();
        Point nearestP2(0, 0);

        for ( auto& p2 : vec2) {
            double dist = p1.calculate_distance(p2);
            if (dist < minDist) {
                minDist = dist;
                nearestP2 = p2;
            }
        }

        Point midpoint((p1.x + nearestP2.x) / 2, (p1.y + nearestP2.y) / 2);
        if (isMidpointUnique(midpoint)) {
            result.emplace_back(p1, nearestP2, minDist);
        }
    }

    for ( auto& p2 : vec2) {
        double minDist = std::numeric_limits<double>::max();
        Point nearestP1(0, 0);

        for ( auto& p1 : vec1) {
            double dist = p2.calculate_distance(p1);
            if (dist < minDist) {
                minDist = dist;
                nearestP1 = p1;
            }
        }

        Point midpoint((p2.x + nearestP1.x) / 2, (p2.y + nearestP1.y) / 2);
        if (isMidpointUnique(midpoint)) {
            result.emplace_back(nearestP1, p2, minDist);
        }
    }

    return result;
};


class MessageBase {
    public:
        static vector<vector<int>> parse_message(nav_msgs::msg::OccupancyGrid m) {
            Config::write_info("frame_id: " + m.header.frame_id);
            Config::write_info("frame_id length: "+ m.header.frame_id.length());
            // if (m.header.frame_id.length() > 1000) {  // Adjust this limit as needed
            //     throw std::length_error("frame_id is too long: " + std::to_string(m.header.frame_id.length()));
            // }
            int h = m.info.height;
            int w = m.info.width;
            if (h <= 0 || w <= 0 || h > 100000 || w > 100000) {
                throw std::runtime_error("Invalid grid dimensions: " + std::to_string(h) + "x" + std::to_string(w));
            }
            if (m.data.size() != static_cast<size_t>(h * w)) {
                throw std::runtime_error("Grid data size mismatch: expected " + std::to_string(h * w) + ", got " + std::to_string(m.data.size()));
            }
            vector<vector<int>> grid(h, vector<int>(w));
            for (int i = 0; i < h; i++) {
                for (int j = 0; j < w; j++) {
                    grid[i][j] = m.data[i * w + j];
                }
            }
            return grid;
        }
        static Point parse_message(geometry_msgs::msg::PoseStamped m) {
            Config::write_info("frame_id: " + m.header.frame_id);
            Config::write_info("frame_id length: "+ m.header.frame_id.length());

            // if (m.header.frame_id.length() > 1000) {  // Adjust this limit as needed
            //     throw std::length_error("PoseStamped frame_id is too long: " + std::to_string(m.header.frame_id.length()));
            // }

            double x = m.pose.position.x;
            double y = m.pose.position.y;
            return Point(x, y);
        }
        static double parse_message(std_msgs::msg::Float64 m) {
            return m.data;
        }
};

template <class T>
class Message : public MessageBase {
public:
    T message;
    Message(T message) : message(message) {}  

    auto parse_message() {
        return parse_message(this->message);
    }  
};


template <class messageDatatype, class parsedDatatype> 
class Subscription {
    public:
        string alias;
        string topic;
        int maxlength;
        bool received;
        deque<messageDatatype> messages;
        static map<std::string, Subscription<messageDatatype, parsedDatatype>*> subs;
        using subdatatype = typename rclcpp::Subscription<messageDatatype>::SharedPtr;
        subdatatype sub;
        Subscription(string alias, string topic, int maxlength, rclcpp::Node* parentNode) {
            this->alias = alias;
            this->topic = topic;
            this->maxlength = maxlength;
            this->sub = parentNode->create_subscription<messageDatatype>(
                topic, 10, std::bind(&Subscription::set_message, this, std::placeholders::_1));
            // subs[alias] = this;
            this->received = false;
        }
        using messageshptr = typename messageDatatype::SharedPtr;
        void set_message(messageshptr message) {
            messages.push_back(*message);
            if(messages.size()>maxlength) {
                messages.pop_front();
            }
            if(this->received == false) {
                this->received = true;
            }
        }
        deque<messageDatatype> get_all_messages() {
            return messages;
        }
        messageDatatype get_latest_message() {
            return messages.back();
        }
        vector<parsedDatatype> get_all_data() {
            vector<parsedDatatype> datas;
            for(messageDatatype& message: messages) {
                datas.push_back(MessageBase::parse_message(message));
            }
            return datas;
        }
        parsedDatatype get_latest_data() {
            return MessageBase::parse_message(messages.back());
        }
};

template <class messageDatatype, class parsedDatatype>
map<string, Subscription<messageDatatype, parsedDatatype>*> Subscription<messageDatatype, parsedDatatype>::subs;

template <class messageDatatype>
class Publishing {
    public:
        static map<std::string, Publishing*> pubs;
        string topic;
        string alias;
        typename rclcpp::Publisher<messageDatatype>::SharedPtr pub;
        Publishing(string alias, string topic, rclcpp::Node* parentNode) {
            this->topic = topic;
            this->alias = alias;
            this->pub = parentNode->create_publisher<messageDatatype>(this->topic, 10);
            // pubs[alias] = this;
        }
};

template <class messageDatatype>
map<string, Publishing<messageDatatype>*> Publishing<messageDatatype>::pubs;


class CandidateGoal : public Point {
    public:
        Point parent1;
        Point parent2;
        double goal_angle;
        double parent_distance;
        double robot_distance;
        double heuristic;
        CandidateGoal(Point p1, Point p2)
            : Point((p1.x+p2.x)/2, (p1.y+p2.y)/2), parent1(p1), parent2(p2) {
            this->robot_distance = calculate_robot_distance();
            this->parent_distance = calculate_parent_distance();
            this->goal_angle = calculate_goal_angle();
            this->heuristic = calculate_heuristic();
        }
    private:
        double calculate_robot_distance() {
            return this->calculate_distance(Subscription<geometry_msgs::msg::PoseStamped,Point>::subs["robot_pose_global"]->get_latest_data());
        }
        double calculate_parent_distance() {
            return this->parent1.calculate_distance(this->parent2);
        }
        double calculate_goal_angle() {
            return this->calculate_angle(Config::goals.end()[-1], Config::goals.end()[-2]);
        }
        bool check_obstacle() {
            vector<vector<int>> map_data = Subscription<nav_msgs::msg::OccupancyGrid,vector<vector<int>>>::subs["map"]->get_latest_data();
            return (map_data[this->y][this->x] > 0);
        }
        bool validate() {
            bool condition = true;
            condition = condition && this->check_obstacle();
            condition = condition && this->robot_distance < Config::config["goal_distance_max"].as<double>();
            condition = condition && this->robot_distance > Config::config["goal_distance_min"].as<double>();
            condition = condition && this->goal_angle > Config::config["goal_angle_threshold"].as<double>();
            condition = condition && this->parent_distance != 0; //to prevent zero division error in heuristic
            return condition;
        }
        double calculate_heuristic() {
            if (this->validate() == false) {
                return -1.0;
            } else {
                return this->robot_distance * this->goal_angle / this->parent_distance;
            }
        }
};

class GoalCalculator : public rclcpp::Node {
    public:
        vector<Point> goals;
        bool start;
        bool shutdown;
        rclcpp::TimerBase::SharedPtr timer_;

        GoalCalculator() : Node("goal_calculator_cpp_node") {
            
            Config::nodeptr = this;

            start = true;
            shutdown = false;

            this->create_sub<nav_msgs::msg::OccupancyGrid , vector<vector<int>>>("map", "/local_map", 1);
            this->create_sub<geometry_msgs::msg::PoseStamped, Point>("robot_pose_grid", "/robot_pose_grid", 1);
            this->create_sub<geometry_msgs::msg::PoseStamped, Point>("robot_pose_global", "/robot_pose_global", 1);
            this->create_sub<std_msgs::msg::Float64, double>("robot_orientation", "/robot_orientation", 10); 
            RCLCPP_INFO(this->get_logger(), "Subscribed to all topics"); 

            this->create_pub<geometry_msgs::msg::PoseStamped>("goal_pose", "/goal_pose");
            RCLCPP_INFO(this->get_logger(), "Publishing to all topics"); 

            timer_ = this->create_wall_timer(
                std::chrono::duration<double>(Config::config["controller_interval"].as<double>()),
                std::bind(&GoalCalculator::controller, this));

            RCLCPP_INFO(this->get_logger(), "Goal calculator initialized"); 
        }
    private:
        template <class messageDatatype, class parsedDatatype>
        void create_sub(const std::string& alias, const std::string& topic, int maxlength) {
            // using subdatatype = typename rclcpp::Subscription<messageDatatype>::SharedPtr;
            Subscription<messageDatatype, parsedDatatype> s = Subscription<messageDatatype, parsedDatatype>(alias, topic, maxlength, this);
            Subscription<messageDatatype, parsedDatatype>::subs[alias] = &s;
            RCLCPP_INFO(this->get_logger(), "Subscribed to '%s'", topic.c_str()); 
        }
        
        template <class messageDatatype>
        void create_pub(const std::string& alias, const std::string& topic) {
            // using pubdatatype = typename rclcpp::Publisher<messageDatatype>::SharedPtr;
            Publishing<messageDatatype> p = Publishing<messageDatatype>(alias, topic, this);
            Publishing<messageDatatype>::pubs[alias] = &p; 
            RCLCPP_INFO(this->get_logger(), "Publishing to '%s'", topic.c_str()); 
        }
        void controller() {
            Config::write_info("Controller is active");
            bool topics_received = true;
            if(Subscription<geometry_msgs::msg::PoseStamped,Point>::subs["robot_pose_global"]->received == false) {
                Config::write_info("robot_pose_global not found");
                topics_received = false;
            }
            if(Subscription<nav_msgs::msg::OccupancyGrid,vector<vector<int>>>::subs["map"]->received == false) {
                Config::write_info("local_map not found");
                topics_received = false;
            }
            if(topics_received == false) {
                Config::write_info("Not enough info to search and publish.");
                return;
            }
            Point robot_coords_global = Subscription<geometry_msgs::msg::PoseStamped,Point>::subs["robot_pose_global"]->get_latest_data();
            vector<vector<int>>map_data = Subscription<nav_msgs::msg::OccupancyGrid,vector<vector<int>>>::subs["map"]->get_latest_data();
            Point robot_coords_grid = Subscription<geometry_msgs::msg::PoseStamped,Point>::subs["robot_pose_grid"]->get_latest_data();            
            Config::write_info("Local map shape: "+to_string(map_data.size())+"x"+to_string(map_data[0].size()));
            if (this->start == true) {
                Config::write_info("Publishing start goal");
                Config::goals.push_back(robot_coords_global);
                this->start = false;
                return;
            }
            this->check_shutdown();
            if (this->shutdown == true) {
                Config::write_info("Goal calculator is shutdown");
                return;
            }
            Config::write_info("Calculating goal pose");
            auto t1 = std::chrono::high_resolution_clock::now();
            std::optional<geometry_msgs::msg::PoseStamped> goal_pose = this->get_goal_pose();
            auto t2 = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double, std::milli> ms_double = t2 - t1;
            auto duration_secs = chrono::duration_cast<chrono::seconds>(ms_double);
            string duration_secs_str = to_string(duration_secs.count());   
            if(!goal_pose.has_value()) {
                Config::write_info("No valid goal pose was found (Time taken: "+duration_secs_str+")");
            } else {
                Config::write_info("Goal pose calculated (Time taken: )"+duration_secs_str+")");
                this->publish_goal(goal_pose.value());
            }
        }
        void check_shutdown() {
            Point shutdown_coords_global = Point(Config::config["shutdown_coords_global"].as<vector<double>>());
            Point resume_coords_global = Point(Config::config["resume_coords_global"].as<vector<double>>());
            double shutdown_tolerance_distance = Config::config["shutdown_tolerance_distance"].as<double>();
            double resume_tolerance_distance = Config::config["resume_tolerance_distance"].as<double>();
            double resume_goal_distance = Config::config["resume_goal_distance"].as<double>();
            std::optional<Point> robot_coords_global = Subscription<geometry_msgs::msg::PoseStamped,Point>::subs["robot_pose_global"]->get_latest_data();
            if(this->shutdown == false && robot_coords_global->calculate_distance(shutdown_coords_global)<shutdown_tolerance_distance) {
                this->shutdown = true;
                Config::write_info("Goal calculator shutdown");
            }
            if(this->shutdown == true && robot_coords_global->calculate_distance(resume_coords_global)<resume_tolerance_distance) {
                this->shutdown = false;
                Config::write_info("Publishing resume goal");
                Config::goals.clear();
                Config::goals.push_back(resume_coords_global);
                geometry_msgs::msg::PoseStamped resume_goal_pose = this->create_goal_pose(Point(resume_coords_global.x-resume_goal_distance, resume_coords_global.y));
                this->publish_goal(resume_goal_pose);
            } 
        }
        geometry_msgs::msg::PoseStamped create_goal_pose(Point goal) {
            geometry_msgs::msg::PoseStamped goal_pose;
            goal_pose.header.stamp = this->get_clock()->now();
            goal_pose.pose.position.x = goal.x;
            goal_pose.pose.position.y = goal.y;

            vector<double> robot_orientation_data = Subscription<std_msgs::msg::Float64,double>::subs["robot_orientation"]->get_all_data();
            double average_orientation = accumulate(robot_orientation_data.begin(), robot_orientation_data.end(), 0.0) / robot_orientation_data.size();
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.0;
            goal_pose.pose.orientation.z = sin(average_orientation);
            goal_pose.pose.orientation.w = cos(average_orientation);

            return goal_pose;
        }

        std::optional<geometry_msgs::msg::PoseStamped> get_goal_pose() {
            /*
            DBScan and get 2 largest groups of points
            Create list of pairs of points with distance between them
            Create list of midpoints
            Filter midpoints
            Pick midpoint with best heuristic
            */
            Message<nav_msgs::msg::OccupancyGrid> enclosed_map_message = Subscription<nav_msgs::msg::OccupancyGrid, vector<vector<int>>>::subs["maps"]->get_latest_message();
            nav_msgs::msg::OccupancyGrid map_message = enclosed_map_message.message;
            vector<vector<int>> map_data = MessageBase::parse_message(map_message);
            double eps = Config::config["dbscan_eps"].as<double>();
            int minPts = Config::config["dbscan_min_samples"].as<int>();
            vector<vector<Point>> clusters = dbscan(map_data, eps, minPts);
            vector<PointPair> nearest_pairs = findNearestPointPairs(clusters.begin()[0], clusters.begin()[1]);
            vector<CandidateGoal> candidate_goals;
            double h;
            double max_h = -1.0;
            int i;
            CandidateGoal best_candidate_goal = CandidateGoal(Point(0.0,0.0),Point(0.0,0.0));
            CandidateGoal candidate_goal = CandidateGoal(Point(0.0,0.0),Point(0.0,0.0));
            bool found = false;
            for(std::vector<PointPair>::size_type i = 0; i != nearest_pairs.size(); i++) {
                CandidateGoal candidate_goal = CandidateGoal(nearest_pairs[i].p1.convert_to_global_coords(map_message), nearest_pairs[i].p2.convert_to_global_coords(map_message));
                h = candidate_goal.heuristic;
                if(h>0) {
                    if(h > max_h) {
                        max_h = h;  
                        CandidateGoal best_candidate_goal = CandidateGoal(candidate_goal.parent1, candidate_goal.parent2);
                        found = true;
                    }
                }
            }
            if(found == true) {
                std::optional<geometry_msgs::msg::PoseStamped> goal_pose = create_goal_pose(best_candidate_goal);
                return goal_pose;
            } else {
                return std::nullopt;
            }
        }    
              
        void publish_goal(geometry_msgs::msg::PoseStamped goal_pose) {
            Point parsed_goal = MessageBase::parse_message(goal_pose);
            Publishing<geometry_msgs::msg::PoseStamped>::pubs["goal_pose"]->pub->publish(goal_pose);
            if(Config::goals.size()<=1 || parsed_goal.calculate_distance(Config::goals.end()[-1])>Config::config["goal_logger_in_between_distance"].as<double>()) {
                Config::goals.push_back(parsed_goal);
            }
            Config::write_info("Published goal");
        }
};


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    Config c = Config();
    auto node = std::make_shared<GoalCalculator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}