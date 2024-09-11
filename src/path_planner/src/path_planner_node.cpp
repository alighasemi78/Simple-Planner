// Required ROS libraries for communication and message handling
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

// Global flags to track if the initial pose and goal pose have been received
bool init_pose_received = false;
bool goal_pose_received = false;

// Global variables to store the received initial pose, goal pose, and the occupancy grid map
geometry_msgs::Pose global_init_pose;
geometry_msgs::Pose global_goal_pose;
nav_msgs::OccupancyGrid::ConstPtr global_map;

// Callback to handle the initial pose message, storing the received pose and setting the flag
void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose.pose;  // Extract the pose from the message
    global_init_pose = pose;  // Store the pose globally
    init_pose_received = true;  // Mark the initial pose as received
    ROS_INFO("Received initial pose: x=%f, y=%f", pose.position.x, pose.position.y);
}

// Callback to handle the goal pose message, storing the received goal pose and setting the flag
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    geometry_msgs::Pose pose = msg->pose;  // Extract the pose from the message
    global_goal_pose = pose;  // Store the goal pose globally
    goal_pose_received = true;  // Mark the goal pose as received
    ROS_INFO("Received goal pose: x=%f, y=%f", pose.position.x, pose.position.y);
}

// Callback to handle the occupancy grid map, storing the map globally
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& map)
{
    global_map = map;  // Store the received map globally
    ROS_INFO("Received Map from map_server, resolution: %lf", map->info.resolution);
}

// Converts grid coordinates (map cells) to world coordinates (meters)
void gridToWorld(double x_grid, double y_grid, int& x_world, int& y_world)
{
    // Convert grid coordinates to world coordinates based on the map origin and resolution
    x_world = (x_grid - global_map->info.origin.position.x) / global_map->info.resolution;
    y_world = (y_grid - global_map->info.origin.position.y) / global_map->info.resolution;
}

// Converts world coordinates (meters) to grid coordinates (map cells)
void worldToGrid(int x_world, int y_world, double& x_grid, double& y_grid)
{
    // Convert world coordinates back to grid coordinates
    x_grid = x_world * global_map->info.resolution + global_map->info.origin.position.x;
    y_grid = y_world * global_map->info.resolution + global_map->info.origin.position.y;
}

// Structure to represent a node in the pathfinding algorithm (A*), storing position and costs
struct Node {
    int x, y;  // Coordinates of the node
    float gCost, hCost;  // gCost is the cost from the start, hCost is the heuristic to the goal
    int parentX, parentY;  // Coordinates of the parent node (for path reconstruction)

    // Constructor to initialize a node with given values
    Node(int x_, int y_, float gCost_ = 0, float hCost_ = 0, int parentX_ = -1, int parentY_ = -1)
        : x(x_), y(y_), gCost(gCost_), hCost(hCost_), parentX(parentX_), parentY(parentY_) {}

    // Returns the total cost (fCost = gCost + hCost)
    float fCost() const {
        return gCost + hCost;
    }

    // Comparison operator to prioritize nodes with lower fCost (for priority queue)
    bool operator<(const Node& other) const {
        return this->fCost() > other.fCost();
    }
};

// Heuristic function for A* (Manhattan distance with a scaling factor for diagonal movement)
float heuristic(int x1, int y1, int x2, int y2) {
    return 1.5 * (abs(x2 - x1) + abs(y2 - y1));
}

// Checks if a given grid coordinate is valid (within bounds and not an obstacle)
bool isValid(int x, int y) {
    int index = y * global_map->info.width + x;  // Convert 2D grid coordinates to a 1D index
    // Ensure the coordinates are within the map bounds and the cell is not occupied (data == 0)
    return x >= 0 && x < global_map->info.width && y >= 0 && y < global_map->info.height && global_map->data[index] == 0;
}

// Computes distances from obstacles for all cells using a breadth-first search (BFS)
std::vector<std::vector<float>> computeObstacleDistances() {
    // Initialize a distance matrix with "infinity" values
    std::vector<std::vector<float>> distances(global_map->info.height, std::vector<float>(global_map->info.width, std::numeric_limits<float>::infinity()));

    // Queue to perform BFS, storing cells with obstacles initially
    std::queue<std::pair<int, int>> q;

    // Find all obstacle cells (occupancy value of 100) and set their distance to 0
    for (int y = 0; y < global_map->info.height; ++y) {
        for (int x = 0; x < global_map->info.width; ++x) {
            int index = y * global_map->info.width + x;
            if (global_map->data[index] == 100) {
                distances[y][x] = 0;
                q.push({x, y});
            }
        }
    }

    // Define the 8 possible directions to move from each cell (including diagonals)
    std::vector<std::pair<int, int>> directions = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};
    
    // BFS to update distances from obstacles
    while (!q.empty()) {
        auto [x, y] = q.front();  // Get the current cell
        q.pop();

        // Explore all 8 neighboring cells
        for (const auto& dir : directions) {
            int newX = x + dir.first;
            int newY = y + dir.second;

            if (isValid(newX, newY)) {  // Ensure the neighbor is valid
                float newDist = distances[y][x] + 1;  // New distance is the distance to the current cell plus 1
                if (newDist < distances[newY][newX]) {  // Update if the new distance is shorter
                    distances[newY][newX] = newDist;
                    q.push({newX, newY});  // Push the neighbor to the queue for further exploration
                }
            }
        }
    }

    return distances;  // Return the matrix of distances
}

// Reconstructs the path from the start to the goal using the parent node information
void reconstructPath(int startX, int startY, int goalX, int goalY,
                     const std::vector<std::vector<int>>& parentX,
                     const std::vector<std::vector<int>>& parentY,
                     ros::Publisher& path_pub) {
    // Marker for visualizing the path in RViz
    visualization_msgs::Marker final_path;
    final_path.header.frame_id = "map";
    final_path.header.stamp = ros::Time::now();
    final_path.ns = "final_path";
    final_path.id = 1;
    final_path.action = visualization_msgs::Marker::ADD;
    final_path.pose.orientation.w = 1.0;
    final_path.type = visualization_msgs::Marker::LINE_STRIP;
    final_path.scale.x = 0.05;  // Line thickness
    final_path.color.r = 0.0;
    final_path.color.g = 0.0;
    final_path.color.b = 1.0;  // Blue color
    final_path.color.a = 1.0;  // Fully opaque

    // Start from the goal and trace back to the start using the parent nodes
    int x = goalX, y = goalY;

    while (x != startX || y != startY) {  // Continue until we reach the start
        geometry_msgs::Point p;
        double x_grid, y_grid;
        worldToGrid(x, y, x_grid, y_grid);  // Convert world coordinates to grid coordinates
        p.x = x_grid;
        p.y = y_grid;
        p.z = 0;
        final_path.points.push_back(p);  // Add the current point to the path

        // Move to the parent node
        int tempX = parentX[y][x];
        int tempY = parentY[y][x];
        x = tempX;
        y = tempY;
    }

    // Add the start point to the path
    geometry_msgs::Point start_point;
    double startX_grid, startY_grid;
    worldToGrid(startX, startY, startX_grid, startY_grid);
    start_point.x = startX_grid;
    start_point.y = startY_grid;
    start_point.z = 0;
    final_path.points.push_back(start_point);

    path_pub.publish(final_path);  // Publish the path for visualization
}

// A* search algorithm to find the optimal path from start to goal
void AStarSearch(int startX, int startY, int goalX, int goalY, ros::Publisher& path_pub) {
    // Compute the distances from obstacles using BFS
    std::vector<std::vector<float>> obstacleDistances = computeObstacleDistances();

    // Priority queue for the open list, sorting nodes by their fCost (gCost + hCost)
    std::priority_queue<Node> openList;
    // Closed list to track which nodes have been processed
    std::vector<std::vector<bool>> closedList(global_map->info.height, std::vector<bool>(global_map->info.width, false));
    // Parent arrays to store the parent coordinates for each node (used for path reconstruction)
    std::vector<std::vector<int>> parentX(global_map->info.height, std::vector<int>(global_map->info.width, -1));
    std::vector<std::vector<int>> parentY(global_map->info.height, std::vector<int>(global_map->info.width, -1));

    // Add the start node to the open list
    Node start(startX, startY, 0, heuristic(startX, startY, goalX, goalY));
    openList.push(start);

    // Directions for moving to neighboring cells (8 directions including diagonals)
    std::vector<std::pair<int, int>> directions = {
        {0, 1}, {0, -1}, {1, 0}, {-1, 0},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    // Main A* search loop
    while (!openList.empty()) {
        Node current = openList.top();  // Get the node with the lowest fCost
        openList.pop();

        // Check if we have reached the goal
        if (current.x == goalX && current.y == goalY) {
            std::cout << "Goal reached!" << std::endl;
            reconstructPath(startX, startY, goalX, goalY, parentX, parentY, path_pub);  // Reconstruct the path
            return;
        }

        // Skip if the current node has already been processed
        if (closedList[current.y][current.x]) {
            continue;
        }

        // Mark the current node as processed
        closedList[current.y][current.x] = true;

        // Explore all neighboring cells
        for (const auto& dir : directions) {
            int newX = current.x + dir.first;
            int newY = current.y + dir.second;

            // Skip invalid or already processed neighbors
            if (!isValid(newX, newY) || closedList[newY][newX]) {
                continue;
            }

            // Calculate the cost to move to the neighbor, considering proximity to obstacles
            float obstacleProximityCost = 1000 / obstacleDistances[newY][newX];  // Penalty for proximity to obstacles
            float tentativeGCost = current.gCost + 1 + obstacleProximityCost;  // Tentative cost from the start

            // Calculate the heuristic cost to the goal
            float hCost = heuristic(newX, newY, goalX, goalY);

            // Add the neighbor to the open list
            openList.push(Node(newX, newY, tentativeGCost, hCost, current.x, current.y));

            // Update the parent of the neighbor to the current node
            parentX[newY][newX] = current.x;
            parentY[newY][newX] = current.y;
        }
    }

    // If the open list is empty and we haven't reached the goal, no path exists
    std::cout << "No path found!" << std::endl;
}

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "path_planner_node");

    // Node handles for subscribing to topics
    ros::NodeHandle init_pose_nh;
    ros::NodeHandle goal_pose_nh;
    ros::NodeHandle map_nh;
    ros::NodeHandle path_nh;

    // Subscribers for initial pose, goal pose, and map topics
    ros::Subscriber init_pose_sub = init_pose_nh.subscribe("/initialpose", 1, initPoseCallback);
    ros::Subscriber goal_pose_sub = goal_pose_nh.subscribe("/move_base_simple/goal", 1, goalPoseCallback);
    ros::Subscriber map_sub = map_nh.subscribe("/map", 1, mapCallback);

    // Publisher for the path visualization marker
    ros::Publisher path_pub = path_nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    // Wait for the map to be received
    while (ros::ok() && !global_map)
    {
        ROS_INFO("Waiting for map");
        ros::Duration(1).sleep();  // Sleep for 1 second between checks
        ros::spinOnce();  // Process any incoming messages
    }

    // Wait for the initial pose to be received
    while (ros::ok() && !init_pose_received)
    {
        ROS_INFO("Waiting for initial pose");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    // Wait for the goal pose to be received
    while (ros::ok() && !goal_pose_received)
    {
        ROS_INFO("Waiting for goal pose");
        ros::Duration(1).sleep();
        ros::spinOnce();
    }

    // Convert the initial and goal world coordinates to grid coordinates
    int start_x_world, start_y_world, goal_x_world, goal_y_world;
    gridToWorld(global_init_pose.position.x, global_init_pose.position.y, start_x_world, start_y_world);
    gridToWorld(global_goal_pose.position.x, global_goal_pose.position.y, goal_x_world, goal_y_world);

    // Perform A* search from the start to the goal
    AStarSearch(start_x_world, start_y_world, goal_x_world, goal_y_world, path_pub);

    ros::spin();  // Keep the node alive and listening for callbacks

    return 0;  // Exit the program
}