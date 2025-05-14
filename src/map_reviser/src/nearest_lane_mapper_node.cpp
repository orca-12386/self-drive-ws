// #define DEBUG

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <cmath>
#include <queue>
#include <array>


class NearestLaneMapperNode : public rclcpp::Node
{
public:
    NearestLaneMapperNode() : 
    rclcpp::Node("nearest_lane_mapper_node")
    {
        RCLCPP_INFO(this->get_logger(), "nearest_lane_mapper_node started");

        this->initialise_data();

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&NearestLaneMapperNode::timer_callback, this));
    };

private:
    void initialise_data() {
        std::string map_sub_topic("/map/white/local");
        std::string map_pub_topic("/map/white/local/near");

        map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_sub_topic, 10, std::bind(&NearestLaneMapperNode::mapCallback, this, std::placeholders::_1));

        odometry_sub = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, 
            std::bind(&NearestLaneMapperNode::odomCallback, this, std::placeholders::_1));

        map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, 10);
        
        map_recv = false;
        odom_recv = false;
        
        nearest_map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        nearest_map_msg->header.frame_id = "map";
    }

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        this->map_msg = msg;
        map_recv = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        this->odometry_msg = msg;
        odom_recv = true;
    }

    void log(std::string str) {
        RCLCPP_INFO(this->get_logger(), str.c_str());
    }

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

    void get_connected_points_bfs(std::array<int,2> src, const nav_msgs::msg::OccupancyGrid::SharedPtr map, std::vector<std::array<int,2>>& connected) {
        std::unordered_set<uint64_t> visited;
        visited.reserve(map->info.width * map->info.height / 10);        
        std::deque<std::array<int, 2>> q;        
        auto hash_coords = [](int x, int y) {
            return (static_cast<uint64_t>(x) << 32) | static_cast<uint64_t>(y);
        };        
        const int width = map->info.width;
        const int height = map->info.height;
        uint64_t src_hash = hash_coords(src[0], src[1]);
        visited.insert(src_hash);
        q.push_back(src);
        // int skip_dist = 25;
        int skip_dist = static_cast<int>(std::round(3/map->info.resolution));
        const int cdx[8] = {1, 0, -1, 0, 1, -1, 1, -1};
        const int cdy[8] = {0, 1, 0, -1, 1, -1, -1, 1};
        int* dx = new int[skip_dist*8];
        int* dy = new int[skip_dist*8];
        int c;
        for(int i = 0; i < skip_dist*8 ; i++) {
            c = (i/8) + 1;
            dx[i] = c*cdx[i%8];
            dy[i] = c*cdy[i%8];
        }
        connected.clear();
        connected.reserve(visited.size() * 2);  
        while(!q.empty()) {
            auto p = q.front();
            q.pop_front();
            connected.push_back(p);
            
            for(int i = 0; i < skip_dist*8; i++) {
                int nx = p[0] + dx[i];
                int ny = p[1] + dy[i];
                
                if(nx < 0 || nx >= width || ny < 0 || ny >= height)
                    continue;
                
                if(map->data[ny * width + nx] <= 0)
                    continue;
                
                uint64_t hash = hash_coords(nx, ny);
                if(visited.find(hash) != visited.end())
                    continue;
                
                visited.insert(hash);
                q.push_back({nx, ny});
            }            
            // if(connected.size() > 10000)
            //     break;
        }
        delete dx;
        delete dy;
    }

    void filter_and_publish_map(const nav_msgs::msg::Odometry::SharedPtr odom, const nav_msgs::msg::OccupancyGrid::SharedPtr map) {
        if(nearest_map_msg->data.size() != map->data.size()) {
            nearest_map_msg->data.resize(map->data.size(), 0);
        } else {
            std::fill(std::begin(nearest_map_msg->data), std::end(nearest_map_msg->data), 0);
        }
        nearest_map_msg->info.width = map->info.width;
        nearest_map_msg->info.height = map->info.height;
        nearest_map_msg->info.resolution = map->info.resolution;
        nearest_map_msg->info.origin.position.x = map->info.origin.position.x;
        nearest_map_msg->info.origin.position.y = map->info.origin.position.y;
        nearest_map_msg->info.origin.position.z = map->info.origin.position.z;
        nearest_map_msg->info.origin.orientation.w = map->info.origin.orientation.w;
        int grid_x = (odom->pose.pose.position.x - map->info.origin.position.x)/map->info.resolution;
        int grid_y = (odom->pose.pose.position.y - map->info.origin.position.y)/map->info.resolution; 
        std::array<int,2> source = {grid_x, grid_y};
        std::array<int,2> nearest_pt;
        // log("finding nearest");
        bool b = get_nearest_point_bfs(source, map, nearest_pt);
        if(!b) {
            map_pub->publish(*map);
            return;
        }
        // log("finding connected");
        std::vector<std::array<int,2>> connected;
        get_connected_points_bfs(nearest_pt, map, connected);
        // log("found connected");
        for(int i=0;i<connected.size();i++) {
            nearest_map_msg->data[connected[i][1]*nearest_map_msg->info.width + connected[i][0]] = 100;
        }
        nearest_map_msg->header.stamp = this->now();
        map_pub->publish(*nearest_map_msg);
        // log("published");
    }

    void timer_callback() {
        if(map_recv && odom_recv) {
            filter_and_publish_map(odometry_msg, map_msg);
        } else {
            log("Waiting for subscriptions");
        }
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub;
    bool map_recv;
    bool odom_recv;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub;
    nav_msgs::msg::OccupancyGrid::SharedPtr map_msg, nearest_map_msg;
    nav_msgs::msg::Odometry::SharedPtr odometry_msg;
    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NearestLaneMapperNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}