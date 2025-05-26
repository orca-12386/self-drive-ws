#include <cmath>
#include <queue>
#include <unordered_set>
#include <vector>

struct WorldPose {
  double x;
  double y;

  WorldPose(double x_val = 0.0, double y_val = 0.0) : x(x_val), y(y_val) {}
};

struct MapPose {
  int x;
  int y;

  MapPose(int x_val = 0, int y_val = 0) : x(x_val), y(y_val) {}
};

struct Map {
  int width, height;
  double resolution;
  WorldPose origin;
  std::vector<std::vector<int>> grid;
};

struct BotPose {
  WorldPose world_pose;
  MapPose map_pose;
  double roll, pitch, yaw;
};

class Utils {
public:
  Utils() {}

  static double getAngleRadians(const WorldPose &a, const WorldPose &b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::atan2(dy, dx);
  }

  static double mapDistance(const MapPose &a, const MapPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  static double worldDistance(const WorldPose &a, const WorldPose &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
  }

  static MapPose
  getMapPoseFromWorldPose(const WorldPose &pose, const Map &map) {
    MapPose map_pose;
    map_pose.x = static_cast<int>((pose.x - map.origin.x) / map.resolution);
    map_pose.y = static_cast<int>((pose.y - map.origin.y) / map.resolution);
    return map_pose;
  }

  static WorldPose
  getWorldPoseFromMapPose(const MapPose &pose, const Map &map) {
    WorldPose world_pose;
    world_pose.x = map.origin.x + (pose.x * map.resolution);
    world_pose.y = map.origin.y + (pose.y * map.resolution);
    return world_pose;
  }

  static MapPose findClosestForValue(
      const MapPose &pose, const Map &map, int radius, int value
  ) {
    const int dx[] = {0, 1, 0, -1, 1, -1, 1, -1};
    const int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};

    std::queue<std::pair<MapPose, int>> q;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    std::unordered_set<int> visited;

    if (pose.x < 0 || pose.x >= map.width || pose.y < 0 ||
        pose.y >= map.height) {
      return MapPose(-1, -1);
    }

    q.push({pose, 0});
    visited.insert(hashFunc(pose));

    while (!q.empty()) {
      auto pair = q.front();
      q.pop();

      MapPose current = pair.first;
      int distance    = pair.second;

      if (map.grid[current.y][current.x] == value) {
        return current;
      }

      if (distance >= radius) {
        continue;
      }

      for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
            visited.find(hash) == visited.end()) {
          visited.insert(hash);
          q.push({neighbor, distance + 1});
        }
      }
    }

    return MapPose(-1, -1);
  }

  static MapPose exploreLane(const MapPose &start, const Map &map, int max_distance = -1) {
    const int dx[] = {0, 1, 0, -1, 1, -1, 1, -1};
    const int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};

    std::queue<MapPose> q;
    std::unordered_set<int> visited;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };

    q.push(start);
    visited.insert(hashFunc(start));

    MapPose last_cell = start;

    while (!q.empty()) {
      MapPose current = q.front();
      q.pop();

      last_cell = current;

      if (max_distance >= 0 && Utils::mapDistance(current, start) > max_distance) {
        continue;
      }

      for (int i = 0; i < 8; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
            visited.find(hash) == visited.end() && map.grid[ny][nx] == 100) {
          visited.insert(hash);
          q.push(neighbor);
        }
      }
    }

    return last_cell;
  }

  static void removeMapBehindBot(
      Map &map, const WorldPose &bot_pose, double angle, int height_to_remove,
      int width_to_remove
  ) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;

    auto process_rows = [&](int start_y, int end_y) {
      for (int y = start_y; y < end_y; y++) {
        for (int x = 0; x < map.width; x++) {
          MapPose map_pose(x, y);
          WorldPose world_pose = Utils::getWorldPoseFromMapPose(map_pose, map);

          double angle_to_pixel = Utils::getAngleRadians(bot_pose, world_pose);

          double angle_diff = angle_to_pixel - angle;

          while (angle_diff > M_PI)
            angle_diff -= 2 * M_PI;
          while (angle_diff < -M_PI)
            angle_diff += 2 * M_PI;

          double dx = world_pose.x - bot_pose.x;
          double dy = world_pose.y - bot_pose.y;

          if (std::abs(angle_diff) > M_PI / 2 &&
              std::abs(dx) <= width_to_remove &&
              std::abs(dy) <= height_to_remove) {
            map.grid[y][x] = -1;
          }
        }
      }
    };

    process_rows(0, map.height);
  }

  static MapPose
  exploreLaneExtended(const MapPose &start, const Map &map, int skip_distance) {
    const int dx[] = {0, 1, 0, -1, 1, -1, 1, -1};
    const int dy[] = {-1, 0, 1, 0, -1, 1, 1, -1};

    std::queue<MapPose> q;
    std::unordered_set<int> visited;

    auto hashFunc = [&map](const MapPose &p) { return p.y * map.width + p.x; };
    auto distance = [](const MapPose &a, const MapPose &b) {
      return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
    };

    q.push(start);
    visited.insert(hashFunc(start));

    MapPose last_cell = start;

    while (!q.empty()) {
      MapPose current = q.front();
      q.pop();

      last_cell = current;

      for (int i = 0; i < 4; i++) {
        int nx = current.x + dx[i];
        int ny = current.y + dy[i];
        MapPose neighbor(nx, ny);
        int hash = hashFunc(neighbor);

        if (nx >= 0 && nx < map.width && ny >= 0 && ny < map.height &&
            visited.find(hash) == visited.end() && map.grid[ny][nx] == 100) {
          visited.insert(hash);
          q.push(neighbor);
        }
      }

      if (q.empty()) {
        int start_x = std::max(0, last_cell.x - skip_distance);
        int start_y = std::max(0, last_cell.y - skip_distance);

        int end_x = std::min(map.width, last_cell.x + skip_distance + 1);
        int end_y = std::min(map.height, last_cell.y + skip_distance + 1);

        for (int j = start_y; j < end_y; j++) {
          for (int i = start_x; i < end_x; i++) {
            MapPose candidate(i, j);
            int hash = hashFunc(candidate);

            if (visited.find(hash) == visited.end() &&
                distance(candidate, last_cell) <= skip_distance &&
                map.grid[j][i] == 100) {
              visited.insert(hash);
              q.push(candidate);
            }
          }
        }
      }
    }

    return last_cell;
  }
};
