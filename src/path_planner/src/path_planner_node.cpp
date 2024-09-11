#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <iostream>
#include <queue>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>

bool init_pose_received = false;
bool goal_pose_received = false;

geometry_msgs::Pose global_init_pose;
geometry_msgs::Pose global_goal_pose;
nav_msgs::OccupancyGrid::ConstPtr global_map;

void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;
    global_init_pose = pose;
    init_pose_received = true;
    ROS_INFO("Received initial pose: x=%f, y=%f", pose.position.x, pose.position.y);
}

void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose;
    global_goal_pose = pose;
    goal_pose_received = true;
    ROS_INFO("Received goal pose: x=%f, y=%f", pose.position.x, pose.position.y);
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    global_map = map;
    ROS_INFO("Received Map from map_server, resolution: %lf", map->info.resolution);
}

void gridToWorld(double x_grid, double y_grid, int& x_world, int& y_world)
{
    x_world = (x_grid - global_map->info.origin.position.x) / global_map->info.resolution;
    y_world = (y_grid - global_map->info.origin.position.y) / global_map->info.resolution;
}

void worldToGrid(int x_world, int y_world, double& x_grid, double& y_grid)
{
    x_grid = x_world * global_map->info.resolution + global_map->info.origin.position.x;
    y_grid = y_world * global_map->info.resolution + global_map->info.origin.position.y;
}

struct Node {
    int x, y;
    float gCost, hCost;
    int parentX, parentY;

    Node(int x_, int y_, float gCost_ = 0, float hCost_ = 0, int parentX_ = -1, int parentY_ = -1)
        : x(x_), y(y_), gCost(gCost_), hCost(hCost_), parentX(parentX_), parentY(parentY_) {}

    float fCost() const {
        return gCost + hCost;
    }

    bool operator<(const Node& other) const {
        return this->fCost() > other.fCost();
    }
};

float heuristic(int x1, int y1, int x2, int y2) {
    return 1.5 * (abs(x2 - x1) + abs(y2 - y1));
}

bool isValid(int x, int y) {
    int index = y * global_map->info.width + x;
    return x >= 0 && x < global_map->info.width && y >= 0 && y < global_map->info.height && global_map->data[index] == 0;
}

std::vector<std::vector<float>> computeObstacleDistances() {
    std::vector<std::vector<float>> distances(global_map->info.height, std::vector<float>(global_map->info.width, std::numeric_limits<float>::infinity()));

    std::queue<std::pair<int, int>> q;

    for (int y = 0; y < global_map->info.height; ++y) {
        for (int x = 0; x < global_map->info.width; ++x) {
            int index = y * global_map->info.width + x;
            if (global_map->data[index] == 100) {
                distances[y][x] = 0;
                q.push({x, y});
            }
        }
    }

    std::vector<std::pair<int, int>> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    
    while (!q.empty()) {
        auto [x, y] = q.front();
        q.pop();

        for (const auto& dir : directions) {
            int newX = x + dir.first;
            int newY = y + dir.second;

            if (isValid(newX, newY)) {
                float newDist = distances[y][x] + 1;
                if (newDist < distances[newY][newX]) {
                    distances[newY][newX] = newDist;
                    q.push({newX, newY});
                }
            }
        }
    }

    return distances;
}

void reconstructPath(int startX, int startY, int goalX, int goalY,
                     const std::vector<std::vector<int>>& parentX,
                     const std::vector<std::vector<int>>& parentY,
                     ros::Publisher& path_pub) {
    visualization_msgs::Marker final_path;
    final_path.header.frame_id = "map";
    final_path.header.stamp = ros::Time::now();
    final_path.ns = "final_path";
    final_path.id = 1;
    final_path.action = visualization_msgs::Marker::ADD;
    final_path.pose.orientation.w = 1.0;
    final_path.type = visualization_msgs::Marker::LINE_STRIP;
    final_path.scale.x = 0.05;
    final_path.color.r = 0.0;
    final_path.color.g = 0.0;
    final_path.color.b = 1.0;
    final_path.color.a = 1.0;

    int x = goalX, y = goalY;

    while (x != startX || y != startY) {
        geometry_msgs::Point p;
        double x_grid, y_grid;
        worldToGrid(x, y, x_grid, y_grid);
        p.x = x_grid;
        p.y = y_grid;
        p.z = 0;
        final_path.points.push_back(p);

        int tempX = parentX[y][x];
        int tempY = parentY[y][x];
        x = tempX;
        y = tempY;
    }

    geometry_msgs::Point start_point;
    double startX_grid, startY_grid;
    worldToGrid(startX, startY, startX_grid, startY_grid);
    start_point.x = startX_grid;
    start_point.y = startY_grid;
    start_point.z = 0;
    final_path.points.push_back(start_point);

    path_pub.publish(final_path);
}

void AStarSearch(int startX, int startY, int goalX, int goalY, ros::Publisher& path_pub) {
    std::vector<std::vector<float>> obstacleDistances = computeObstacleDistances();

    std::priority_queue<Node> openList;
    std::vector<std::vector<bool>> closedList(global_map->info.height, std::vector<bool>(global_map->info.width, false));
    std::vector<std::vector<int>> parentX(global_map->info.height, std::vector<int>(global_map->info.width, -1));
    std::vector<std::vector<int>> parentY(global_map->info.height, std::vector<int>(global_map->info.width, -1));

    Node start(startX, startY, 0, heuristic(startX, startY, goalX, goalY));
    openList.push(start);

    std::vector<std::pair<int, int>> directions = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    while (!openList.empty()) {
        Node current = openList.top();
        openList.pop();

        if (current.x == goalX && current.y == goalY) {
            std::cout << "Goal reached!" << std::endl;
            reconstructPath(startX, startY, goalX, goalY, parentX, parentY, path_pub);
            return;
        }

        if (closedList[current.y][current.x]) {
            continue;
        }

        closedList[current.y][current.x] = true;

        for (const auto& dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;

            if (!isValid(newX, newY) || closedList[newY][newX]) {
                continue;
            }

            float obstacleProximityCost = 1000 / obstacleDistances[newY][newX];
            float tentativeGCost = current.gCost + 1 + obstacleProximityCost;

            float hCost = heuristic(newX, newY, goalX, goalY);

            openList.push(Node(newX, newY, tentativeGCost, hCost, current.x, current.y));

            parentX[newY][newX] = current.x;
            parentY[newY][newX] = current.y;
        }
    }

    std::cout << "No path found!" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "path_planner_node");

    ros::NodeHandle init_pose_nh;
    ros::NodeHandle goal_pose_nh;
    ros::NodeHandle map_nh;
    ros::NodeHandle path_nh;

    ros::Subscriber init_pose_sub = init_pose_nh.subscribe("/initialpose", 1, initPoseCallback);
    ros::Subscriber goal_pose_sub = goal_pose_nh.subscribe("/move_base_simple/goal", 1, goalPoseCallback);
    ros::Subscriber map_sub = map_nh.subscribe("/map", 1, mapCallback);

    ros::Publisher path_pub = path_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    while (ros::ok() && !global_map)
    {
        ROS_INFO("Waiting for map");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    while (ros::ok() && !init_pose_received)
    {
        ROS_INFO("Waiting for initial pose");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    while (ros::ok() && !goal_pose_received)
    {
        ROS_INFO("Waiting for goal pose");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    int start_x_world, start_y_world, goal_x_world, goal_y_world;

    gridToWorld(global_init_pose.position.x, global_init_pose.position.y, start_x_world, start_y_world);
    gridToWorld(global_goal_pose.position.x, global_goal_pose.position.y, goal_x_world, goal_y_world);

    AStarSearch(start_x_world, start_y_world, goal_x_world, goal_y_world, path_pub);

    ros::spin();

    return 0;
}
