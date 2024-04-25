#include "prm.hpp"
#include <pluginlib/class_list_macros.h>
#include <limits>
#include <math.h>
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
            expand_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree",1);
            make_plan_srv = private_nh.advertiseService("make_plan", &PRM::makePlanService, this);

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
        ROS_INFO("Checking if start and goal poses are in the global costmap.");
        double worldX = start.pose.position.x;
        double worldY = start.pose.position.y;
        double mapStartX, mapStartY, mapGoalX, mapGoalY;
        if(!world2Map(worldX, worldY, mapStartX, mapStartY)){
            ROS_WARN_THROTTLE(1.0, "The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");            
            return false;
        }

        worldX = goal.pose.position.x;
        worldY = goal.pose.position.y;
        if(!world2Map(worldX,worldY,mapGoalX, mapGoalY)){
            ROS_WARN_THROTTLE(1.0, "The goal sent to the navfn planner is off the global costmap. Planning will always fail to this goal.");
            return false;
        }

        //transform from costmap to gridmap
        ROS_INFO("Converting from map to grid values.");
        int gridStartX, gridStartY, gridGoalX, gridGoalY;
        map2Grid(mapStartX, mapStartY, gridStartX, gridStartY);
        map2Grid(mapGoalX, mapGoalY, gridGoalX, gridGoalY);


        ROS_INFO("Creating start and goal nodes.");
        Node start_node(gridStartX, gridStartY, 0, grid2Index(gridStartX, gridStartY), 0);
        Node goal_node(gridGoalX, gridGoalY, 0, grid2Index(gridGoalX, gridGoalY), 0);
        
        costmap_->setCost(gridStartX, gridStartY, costmap_2d::FREE_SPACE);
        costmap_->setCost(gridGoalX, gridGoalY, costmap_2d::FREE_SPACE);

        ROS_INFO("Creating path and expand vectors.");

        // calculate path
        std::vector<Node> path, expand;
        bool found_legal = false;

        ROS_INFO("Running PRM");
        found_legal = runPRM(costmap_->getCharMap(), start_node, goal_node, path, expand);

        if (found_legal){
            ROS_INFO("Getting plan from path");
            if(getPlanFromPath(path, plan)){
                ROS_INFO("Plan received");
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

    bool PRM::runPRM(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand){
        ROS_INFO("PRM started");
        path.clear();
        expand.clear();

        // Parameters for PRM algorithm
        const int num_samples = 2000; // Number of random samples
        const int num_neighbors = 20; // Number of nearest neighbors to consider
 
        // Generate random samples in the free space
        std::vector<Node> samples;
        if (samples.empty()) {
            generateRandomSamples(global_costmap, num_samples, samples);
            ROS_INFO("%zu samples generated", samples.size());

            // std::cout << "Nodes in samples:" << std::endl;
            // for (size_t i = 0; i < samples.size(); ++i) {
            //     const Node& node = samples[i];
            //     std::cout << "Index: " << i << ", x: " << node.x_ << ", y: " << node.y_ << std::endl;
            // }
        }
        
        // Connect the samples to create the roadmap
        std::unordered_map<int, std::vector<int>> roadmap;
        if (roadmap.empty()) {
            constructRoadmap(global_costmap, samples, num_neighbors, roadmap);
            ROS_INFO("roadmap constructed");

            // Print the contents of the roadmap
            // std::cout << "Roadmap Contents:" << std::endl;
            // for (const auto& pair : roadmap) {
            //     std::cout << pair.first << ": ";
            //     for (int value : pair.second) {
            //         std::cout << value << " ";
            //     }
            //     std::cout << std::endl;
            // }
        }

        // Find the nearest nodes to the start and goal positions
        int start_index = findNearestNode(start, samples);
        int goal_index = findNearestNode(goal, samples);
        ROS_INFO("Start index: %d, Goal index: %d", start_index, goal_index);

        // Check if start and goal nodes are valid
        if (start_index == -1 || goal_index == -1) {
            ROS_ERROR("Invalid start or goal node.");
            return false;
        }

        // Find a path using A* algorithm on the roadmap
        bool found_path = searchPath(start_index, goal_index, roadmap, samples, path, expand);
        return found_path;
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

    // bool PRM::collisionCheck(const Node& n1, const Node& n2, const unsigned char* global_costmap){
    //     double theta = math::angle(n1, n2);
    //     double dist = math::euclidean_distance(n1, n2);

    //     if(dist > 10){
    //         return true;
    //     }

    //     int step = (int)(dist/resolution);
    //     for(int i = 0; i < step; i++){
    //         float x_step = (float)n1.x_ + (float)(i*resolution*cos(theta));
    //         float y_step = (float)n1.y_ + (float)(i*resolution*sin(theta));
    //         if(global_costmap[grid2Index((int)x_step, (int)y_step)] >= 253*0.5){
    //             return true;
    //         }
    //     }
    //     return false;

    // }

    bool PRM::goalFound(const Node& node, const Node& goal){
        auto dist = euclideanDistance(node, goal);
        if (dist > 10){
            return false;
        }
        else{
            return true;
        }   
    }

    bool PRM::getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized.");
            return false;
        }
        ros::Time planTime = ros::Time::now();
        plan.clear();

        for (int i = path.size() - 1; i >=0; i--){
            double worldX, worldY;
            map2World((double)path[i].x_, (double)path[i].y_, worldX, worldY);
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = frame_id_;
            pose.pose.position.x = worldX;
            pose.pose.position.y = worldY;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            plan.push_back(pose);
        }

        return !plan.empty();

    }

    void PRM::publishExpand(std::vector<Node>& expand){
        ROS_DEBUG("Expand zone size: %1ld", expand.size());

        nav_msgs::OccupancyGrid grid;

        // build expand
        grid.header.frame_id = frame_id_;
        grid.header.stamp = ros::Time::now();
        grid.info.resolution = resolution;
        grid.info.width = width;
        grid.info.height = height;

        double worldX, worldY;
        costmap_->mapToWorld(0,0,worldX,worldY);
        grid.info.origin.position.x = worldX - resolution/2;
        grid.info.origin.position.y = worldY - resolution/2;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        grid.data.resize(width*height);

        for(unsigned int i = 0; i < grid.data.size(); i++){
            grid.data[i] = 0;
        }
        for(unsigned int i = 0; i < expand.size();i++){
            grid.data[expand[i].index_] = 50;
        }
        expand_pub_.publish(grid);
    }

    void PRM::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
        if(!initialized_){
            ROS_ERROR("This planner has not been initialized.");
            return;
        }
        nav_msgs::Path gui_plan;
        gui_plan.poses.resize(plan.size());
        gui_plan.header.frame_id = frame_id_;
        gui_plan.header.stamp = ros::Time::now();
        for (unsigned int i = 0; i < plan.size(); i++){
            gui_plan.poses[i] = plan[i];
        }
        plan_pub_.publish(gui_plan);
    }

    void PRM::generateRandomSamples(const unsigned char* global_costmap, int num_samples, std::vector<Node>& samples) {
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
        ROS_INFO("constructing roadmap");

        for (size_t i = 0; i < samples.size(); ++i) {
            std::vector<int> neighbors;
            for (size_t j = 0; j < samples.size(); ++j) {
                if (i != j) {
                    double dist = euclideanDistance(samples[i], samples[j]);

                    // Check if the distance is within the specified range and there is a clear path between the nodes
                    if (dist <= max_distance && isClearPath(global_costmap, samples[i], samples[j])) {
                        neighbors.push_back(j);

                        // Onto the next node if we have found enough neighbors
                        if (neighbors.size() == num_neighbors) {
                            break;
                        }
                    }
                }
            
            }
            roadmap[i] = neighbors;
        }
    }

    bool PRM::isClearPath(const unsigned char* global_costmap, const Node& node1, const Node& node2) {
        int width = costmap_->getSizeInCellsX();
        int height = costmap_->getSizeInCellsY();

        int x0 = node1.x_, y0 = node1.y_;
        int x1 = node2.x_, y1 = node2.y_;

        int dx = abs(x1 - x0), dy = abs(y1 - y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;

        while (x0 != x1 || y0 != y1) {
            if (global_costmap[x0 + y0 * width] > 25) {
                return false;
            }
            e2 = err;
            if (e2 > -dx) { err -= dy; x0 += sx; }
            if (e2 < dy) { err += dx; y0 += sy; }
        }
        return true;
    }

    bool PRM::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;

        return true;
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

            // expand contains ALL nodes searched, not the final path
            expand.emplace_back(samples[current_index]);

            // have we reached the goal?
            if (goalFound(samples[current_index], samples[goal_index])) {
                ROS_INFO("Goal found! Reconstructing path.");
                // Reconstruct the path
                int current = goal_index;
                int iterations = 0;
                while (current != start_index && iterations < 10000) {
                    path.emplace_back(samples[current]);
                    current = came_from[current];
                    ++iterations;
                }
                // don't search for a path forever
                if (iterations == 10000) {
                    return false;
                }
                path.emplace_back(samples[start_index]);
                std::reverse(path.begin(), path.end());
                return true;
            }

            // check all the neighbors of the current node
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

    double PRM::euclideanDistance(const Node& node1, const Node& node2) {
        double dx = node1.x_ - node2.x_;
        double dy = node1.y_ - node2.y_;
        return sqrt(dx * dx + dy * dy);
    }

    bool PRM::world2Map(double worldX, double worldY, double& mapX, double& mapY){
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

    void PRM::map2World(double mapX, double mapY, double& worldX, double& worldY){
        worldX = originX + (mapX + convert_offset) * resolution;
        worldY = originY + (mapY + convert_offset) * resolution;
    }

    void PRM::map2Grid(double mapX, double mapY, int& gridX, int& gridY){
        gridX = (int)mapX;
        gridY = (int)mapY;
    }

    int PRM::grid2Index(int x, int y){        
        return x + width * y;
    }

};
