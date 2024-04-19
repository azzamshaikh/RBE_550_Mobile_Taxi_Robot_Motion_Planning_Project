#include "prm.hpp"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(PRM_Planner::PRM, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace PRM_Planner {

    // Default constructor
    PRM::PRM() : initialized_(false), costmap_(nullptr) {}

    // Constructor with cost map
    PRM::PRM(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void PRM::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros; // Initialize the costmap_ros_ attribute to the parameter
            costmap_ = costmap_ros_->getCostmap();   // Get the costmap_ from costmap_ros_
            frame_id_ = costmap_ros_->getGlobalFrameID();

            // Initialize other planner parameters
            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);
            expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand",1);
            //make_plan_srv = private_nh.advertiseService("make_plan", &PRM::makePlanService, this);


            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            convert_offset = 0.0;

            mapSize = width*height;

            ROS_INFO("PRM Global Planner initialized successfully.");
            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing.");
        }
    }

    bool PRM::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan ){
        
        if (!initialized_){
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        plan.clear();

        ROS_INFO("PRM Global Planner");
        ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
        ROS_INFO("Goal Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

        if (goal.header.frame_id != frame_id_)
        {
            ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                    frame_id_.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        if (start.header.frame_id != frame_id_)
        {
            ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", 
                    frame_id_.c_str(), start.header.frame_id.c_str());
            return false;
        }

        // Convert the start and goal positions
        // ROS_INFO("Checking if start and goal poses are in the global costmap.");
        double worldX = start.pose.position.x;
        double worldY = start.pose.position.y;
        unsigned int mapStartX, mapStartY, mapGoalX, mapGoalY;
        if(!worldToMap(worldX,worldY,mapStartX, mapStartY)){
            ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");            
            return false;
        }

        worldX = goal.pose.position.x;
        worldY = goal.pose.position.y;
        if(!worldToMap(worldX,worldY,mapGoalX, mapGoalY)){
            ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }

        //transform from costmap to gridmap
        // ROS_INFO("Converting from map to grid values.");
        int gridStartX, gridStartY, gridGoalX, gridGoalY;
        map2Grid(mapStartX,mapStartY,gridStartX, gridStartY);
        map2Grid(mapGoalX, mapGoalY,gridGoalX, gridGoalY);

        // ROS_INFO("Creating start and goal nodes.");

        Node start_node(gridStartX,gridStartY,0,grid2Index(gridStartX,gridStartY),0);
        Node goal_node(gridGoalX,gridGoalY,0,grid2Index(gridGoalX,gridGoalY),0);

        // ROS_INFO("Creating path and expand vectors.");

        // calculate path
        std::vector<Node> path, expand;
        bool found_legal = false;

        // ROS_INFO("Running PRM");
        found_legal = runPRM(costmap_->getCharMap(),start_node, goal_node, path, expand);

        if (found_legal){
            if(getPlanFromPath(path, plan)){
                geometry_msgs::PoseStamped goalCopy = goal;
                goalCopy.header.stamp = ros::Time::now();
                plan.push_back(goalCopy);
            }
            else{
                ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
            }
        }
        else{
            ROS_ERROR("Failed to get a path.");
        }

        publishExpand(expand);

        publishPlan(plan);

        return !plan.empty();
    }

    bool PRM::runPRM(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand) {
    path.clear();
    expand.clear();

    // Parameters for PRM algorithm
    const int num_samples = 100; // Number of random samples
    const int num_neighbors = 5; // Number of nearest neighbors to consider

    // Generate random samples in the free space
    std::vector<Node> samples;
    generateRandomSamples(global_costmap, num_samples, samples);

    // Connect the samples to create the roadmap
    std::unordered_map<int, std::vector<int>> roadmap;
    constructRoadmap(global_costmap, samples, num_neighbors, roadmap);

    // Find the nearest nodes to the start and goal positions
    int start_index = findNearestNode(start, samples);
    int goal_index = findNearestNode(goal, samples);

    // Check if start and goal nodes are valid
    if (start_index == -1 || goal_index == -1) {
        ROS_ERROR("Invalid start or goal node.");
        return false;
    }

    // Find a path using A* algorithm on the roadmap
    bool found_path = searchPath(start_index, goal_index, roadmap, samples, path, expand);

    return found_path;
}

void PRM::generateRandomSamples(const unsigned char* global_costmap, int num_samples, std::vector<Node>& samples) {
    // Get the dimensions of the costmap
    int width = costmap_->getSizeInCellsX();
    int height = costmap_->getSizeInCellsY();

    // Generate random samples within the bounds of the costmap
    for (int i = 0; i < num_samples; ++i) {
        int x = rand() % width;
        int y = rand() % height;

        // Check if the sample is in free space
        if (global_costmap[x + y * width] == 0) {
            samples.emplace_back(x, y);
        }
    }
}

void PRM::constructRoadmap(const unsigned char* global_costmap, const std::vector<Node>& samples, int num_neighbors, std::unordered_map<int, std::vector<int>>& roadmap) {
    // For each sample, find its nearest neighbors and connect them if the path is obstacle-free
    for (size_t i = 0; i < samples.size(); ++i) {
        std::vector<int> neighbors;
        for (size_t j = 0; j < samples.size(); ++j) {
            if (i != j) {
                double dist = euclideanDistance(samples[i], samples[j]);
                // Check if the distance is within the specified range and there is a clear path between the nodes
                if (dist <= max_distance && isClearPath(global_costmap, samples[i], samples[j])) {
                    neighbors.push_back(j);
                }
            }
        }
        roadmap[i] = neighbors;
    }
}

int PRM::findNearestNode(const Node& query, const std::vector<Node>& samples) {
    int nearest_index = -1;
    double min_distance = std::numeric_limits<double>::max();

    // Find the index of the nearest node among the samples
    for (size_t i = 0; i < samples.size(); ++i) {
        double dist = euclideanDistance(query, samples[i]);
        if (dist < min_distance) {
            min_distance = dist;
            nearest_index = i;
        }
    }

    return nearest_index;
}

bool PRM::searchPath(int start_index, int goal_index, const std::unordered_map<int, std::vector<int>>& roadmap, const std::vector<Node>& samples, std::vector<Node>& path, std::vector<Node>& expand) {
    std::unordered_map<int, double> cost_so_far;
    std::unordered_map<int, int> came_from;
    std::priority_queue<std::pair<int, double>, std::vector<std::pair<int, double>>, std::greater<std::pair<int, double>>> frontier;

    frontier.push(std::make_pair(start_index, 0));
    cost_so_far[start_index] = 0;

    while (!frontier.empty()) {
        int current_index = frontier.top().first;
        frontier.pop();

        if (current_index == goal_index) {
            // Reconstruct the path
            int current = goal_index;
            while (current != start_index) {
                path.push_back(samples[current]);
                current = came_from[current];
            }
            path.push_back(samples[start_index]);
            std::reverse(path.begin(), path.end());
            return true;
        }

        for (int next_index : roadmap.at(current_index)) {
            double new_cost = cost_so_far[current_index] + euclideanDistance(samples[current_index], samples[next_index]);
            if (!cost_so_far.count(next_index) || new_cost < cost_so_far[next_index]) {
                cost_so_far[next_index] = new_cost;
                double priority = new_cost + euclideanDistance(samples[next_index], samples[goal_index]);
                frontier.push(std::make_pair(next_index, priority));
                came_from[next_index] = current_index;
            }
        }
    }

    return false; // No path found
}

bool PRM::worldToMap(double worldX, double worldY, unsigned int& mapX, unsigned int& mapY){
        if (worldX < originX || worldY < originY){
            return false;
        }

        mapX = (worldX-originX) / resolution - convert_offset; 
        mapY = (worldY-originY) / resolution - convert_offset; 
        if (mapX < width && mapY < height){
            return true;
        }
        return false;

    }
double PRM::euclideanDistance(const Node& node1, const Node& node2) {
    double dx = node1.x_ - node2.x_;
    double dy = node1.y_ - node2.y_;
    return sqrt(dx * dx + dy * dy);
}

bool PRM::isClearPath(const unsigned char* global_costmap, const Node& node1, const Node& node2) {
    int x0 = node1.x_, y0 = node1.y_;
    int x1 = node2.x_, y1 = node2.y_;

    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = (dx > dy ? dx : -dy) / 2, e2;

    while (x0 != x1 || y0 != y1) {
        if (global_costmap[x0 + y0 * width] != 0) {
            return false;
        }
        e2 = err;
        if (e2 > -dx) { err -= dy; x0 += sx; }
        if (e2 < dy) { err += dx; y0 += sy; }
    }
    return true;
}



    
};
