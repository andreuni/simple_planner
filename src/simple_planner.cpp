#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <queue>
#include <vector>
#include <cmath>
#include <limits>

// Global flags to track received data
bool map_received = false;
bool start_received = false;
bool goal_received = false;

// Global variables to store map and robot positions
nav_msgs::OccupancyGrid map_data;
ros::Publisher marker_pub;
int startX, startY;
int goalX, goalY;

// Structure to represent a search node for Dijkstra's algorithm
struct Node {
    int x, y; // Grid coordinates
    float gCost; // Cumulative cost from start node
    Node* parent; // Pointer to parent node for path reconstruction

    // Constructor to initialize node properties
    Node(int x_, int y_, float gCost_, Node* parent_ = nullptr)
        : x(x_), y(y_), gCost(gCost_), parent(parent_) {}
};

// Comparator for priority queue 
struct CompareNodes {
    bool operator()(Node* a, Node* b) {
        return a->gCost > b->gCost;
    }
};

// Function to check if a cell is valid (not an obstacle and within map bounds)
bool isValid(int x, int y) {
    return x >= 0 && y >= 0 &&
           x < map_data.info.width && y < map_data.info.height && // the point is inside the map
           map_data.data[y * map_data.info.width + x] < 20; // Occupancy threshold
}

// heuristic function 
float euclideanDistance(int x1, int y1, int x2, int y2) {
    return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

// Dijkstra
std::vector<std::pair<int, int>> dijkstraPath() {
    // Check if start and goal positions are valid
    if (!isValid(startX, startY)) {
        ROS_WARN("Please choose a valid START position!");
        return {};
    }
    if (!isValid(goalX, goalY)) {
        ROS_WARN("Please choose a valid GOAL position!");
        return {};
    }

    // Priority queue for Dijkstra
    std::priority_queue<Node*, std::vector<Node*>, CompareNodes> openSet;
    // Closed set to track visited nodes
    std::vector<std::vector<bool>> closedSet(map_data.info.height, std::vector<bool>(map_data.info.width, false));

    // Push the start node into the open set
    openSet.push(new Node(startX, startY, 0));

    std::vector<std::pair<int, int>> path; // Store the final path

    // Main loop for Dijkstra's algorithm
    while (!openSet.empty()) {
        // Get the node with the lowest cost
        Node* current = openSet.top();
        openSet.pop();

        // Check if the goal is reached
        if (current->x == goalX && current->y == goalY) {
            // Backtrack to reconstruct the path
            while (current) {
                path.emplace_back(current->x, current->y);
                current = current->parent;
            }
            std::reverse(path.begin(), path.end()); // Reverse to get correct order
            return path;
        }

        // Mark current node as visited
        if (closedSet[current->y][current->x]) continue;
        closedSet[current->y][current->x] = true;

        // Explore neighbors in 8 directions (including diagonals)
        for (const auto& dir : std::vector<std::pair<int, int>>{
            {0,1}, {1,0}, {0,-1}, {-1,0}, {1,1}, {-1,1}, {1,-1}, {-1,-1}}) {
            
            int nx = current->x + dir.first;
            int ny = current->y + dir.second;

            // Ensure the neighbor is valid and not already visited
            if (isValid(nx, ny) && !closedSet[ny][nx]) {
                float cost = current->gCost + euclideanDistance(current->x, current->y, nx, ny);
                openSet.push(new Node(nx, ny, cost, current)); // Push new node into priority queue
            }
        }
    }

    ROS_WARN("No valid path found!");
    return {};
}

// Publishes the planned path as a marker in RViz
void publishMarkerPath() {
    std::vector<std::pair<int, int>> path = dijkstraPath();
    if (path.empty()) return; // If no valid path, do nothing

    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.header.stamp = ros::Time::now();
    path_marker.ns = "path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.05; 
    path_marker.color.r = 0.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 1.0;
    path_marker.color.a = 1.0;

    // Convert grid coordinates to world coordinates and add to path marker
    for (const auto& point : path) {
        geometry_msgs::Point p;
        p.x = point.first * map_data.info.resolution + map_data.info.origin.position.x;
        p.y = point.second * map_data.info.resolution + map_data.info.origin.position.y;
        p.z = 0.0;
        path_marker.points.push_back(p);
    }

    marker_pub.publish(path_marker);
    ROS_INFO("Path published!");
}

// marker for the robot's start position
void publishStartMarker() {
    if (!start_received) return; // Skip if start position not set

    visualization_msgs::Marker start_marker;
    start_marker.header.frame_id = "map";
    start_marker.header.stamp = ros::Time::now();
    start_marker.ns = "robot";
    start_marker.id = 1;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;

    // Set position based on start grid coordinates
    start_marker.pose.position.x = startX * map_data.info.resolution + map_data.info.origin.position.x;
    start_marker.pose.position.y = startY * map_data.info.resolution + map_data.info.origin.position.y;
    start_marker.pose.position.z = 0.03; 

    // Set sphere size
    start_marker.scale.x = 0.3;
    start_marker.scale.y = 0.3;
    start_marker.scale.z = 0.3;

    // Set color
    start_marker.color.r = 1.0;
    start_marker.color.g = 0.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;

    marker_pub.publish(start_marker);
}

// marker for the robot's goal position
void publishGoalMarker() {
    if (!goal_received) return; // Skip if start position not set

    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "map";
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "goal";
    goal_marker.id = 2;
    goal_marker.type = visualization_msgs::Marker::CUBE;
    goal_marker.action = visualization_msgs::Marker::ADD;

    // Set position based on goal grid coordinates
    goal_marker.pose.position.x = goalX * map_data.info.resolution + map_data.info.origin.position.x;
    goal_marker.pose.position.y = goalY * map_data.info.resolution + map_data.info.origin.position.y;
    goal_marker.pose.position.z = 0.04; 

    // cube size
    goal_marker.scale.x = 0.2;
    goal_marker.scale.y = 0.2;
    goal_marker.scale.z = 0.2;

    // color
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    goal_marker.color.b = 1.0;
    goal_marker.color.a = 1.0;

    marker_pub.publish(goal_marker);
}

// Callback function for handling map updates
void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    ROS_INFO("Received map data");
    map_data = *msg;
    map_received = true;
}

// Callback function for receiving the robot's initial position
void startPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    startX = (msg->pose.pose.position.x - map_data.info.origin.position.x) / map_data.info.resolution;
    startY = (msg->pose.pose.position.y - map_data.info.origin.position.y) / map_data.info.resolution;
    start_received = true;
}

// Callback function for receiving the goal position
void goalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goalX = (msg->pose.position.x - map_data.info.origin.position.x) / map_data.info.resolution;
    goalY = (msg->pose.position.y - map_data.info.origin.position.y) / map_data.info.resolution;
    goal_received = true;
}

// Main function to initialize ROS and manage the node's execution
int main(int argc, char** argv) {
    ros::init(argc, argv, "dijkstra_planner");
    ros::NodeHandle nh;

    // Initialize publisher and subscribers
    marker_pub = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    ros::Subscriber map_sub = nh.subscribe("/map", 1, mapCallback);
    ros::Subscriber start_sub = nh.subscribe("/initialpose", 1, startPoseCallback);
    ros::Subscriber goal_sub = nh.subscribe("/move_base_simple/goal", 1, goalPoseCallback);

    ros::Rate rate(1);

    while (ros::ok()) {
        publishMarkerPath();
        publishStartMarker();
        publishGoalMarker();
        ros::spinOnce(); // Process incoming messages
        rate.sleep();
    }

    return 0;
}