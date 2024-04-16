#include "dijkstra.hpp"
#include <pluginlib/class_list_macros.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(Dijkstra_Planner::Dijkstra, nav_core::BaseGlobalPlanner)

using namespace std;


//Default Constructor
namespace Dijkstra_Planner {

    // Default constructor
    Dijkstra::Dijkstra() : initialized_(false), costmap_(nullptr) {}

    // Constructor with cost map
    Dijkstra::Dijkstra(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void Dijkstra::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros; // Initialize the costmap_ros_ attribute to the parameter
            
            // Initialize other planner parameters
            ros::NodeHandle private_nh("~/" + name);

            costmap_ = costmap_ros_->getCostmap();   // Get the costmap_ from costmap_ros_

            frame_id_ = costmap_ros_->getGlobalFrameID();

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            convert_offset = 0.0;

            mapSize = width*height;

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);

            expand_pub_ = private_nh.advertise<nav_msgs::OccupancyGrid>("expand",1);

            make_plan_srv = private_nh.advertiseService("make_plan", &Dijkstra::makePlanService, this);

            ROS_INFO("Dijkstra Global Planner initialized successfully.");
            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing.");
        }
    }

    bool Dijkstra::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan ){
        
        if (!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        plan.clear();

        ROS_INFO("Dijkstra Global Planner");
        ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
        ROS_INFO("Goal Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

        if (goal.header.frame_id != frame_id_)
        {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                    frame_id_.c_str(), goal.header.frame_id.c_str());
            return false;
        }

        if (start.header.frame_id != frame_id_)
        {
            ROS_ERROR("This planner as configured will only accept start pose in the %s frame, but a start pose was sent in the %s frame.",
                    frame_id_.c_str(), start.header.frame_id.c_str());
            return false;
        }


        // Convert the start and goal positions
        // ROS_INFO("Checking if start and goal poses are in the global costmap.");
        double worldX = start.pose.position.x;
        double worldY = start.pose.position.y;
        double mapStartX, mapStartY, mapGoalX, mapGoalY;
        if(!world2Map(worldX,worldY,mapStartX, mapStartY)){
            ROS_WARN("THE start pose is off the global costmap.");
            return false;
        }

        worldX = goal.pose.position.x;
        worldY = goal.pose.position.y;
        if(!world2Map(worldX,worldY,mapGoalX, mapGoalY)){
            ROS_WARN_THROTTLE(1.0,"The goal pose is off the global costmap.");
            return false;
        }

        //transform from costmap to gridmap
        // ROS_INFO("Converting from map to grid values.");
        int gridStartX, gridStartY, gridGoalX, gridGoalY;
        map2Grid(mapStartX,mapStartY,gridStartX, gridStartY);
        map2Grid(mapGoalX, mapGoalY,gridGoalX, gridGoalY);


        // ROS_INFO("Creating start and goal nodes.");

        Node start_node(gridStartX,gridStartY,0,0,grid2Index(gridStartX,gridStartY),0);
        Node goal_node(gridGoalX,gridGoalY,0,0,grid2Index(gridGoalX,gridGoalY),0);
        
        // outline map????
        // outlineMap(costmap_->getCharMap());

        // ROS_INFO("Creating path and expand vectors.");

        // calculate path
        std::vector<Node> path;
        std::vector<Node> expand;
        bool path_found = false;

        // ROS_INFO("Running A*");
        // planning
        path_found = runDijkstra(costmap_->getCharMap(),start_node, goal_node, path, expand);

        if (path_found){
            if(getPlanFromPath(path, plan)){
                geometry_msgs::PoseStamped goalCopy = goal;
                goalCopy.header.stamp = ros::Time::now();
                plan.push_back(goalCopy);
            }
            else{
                ROS_ERROR("Failed to find path when a legal path is there. This shouldnt happen");
            }
        }
        else{
            ROS_ERROR("Failed to get a path.");
        }

        publishExpand(expand);

        publishPlan(plan);

        return !plan.empty();
    }

    bool Dijkstra::runDijkstra(const unsigned char* global_costmap,const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand){
        path.clear();
        expand.clear();

        //open list and closed list
        std::priority_queue<Node, std::vector<Node>, Node::compare_cost> openList;
        std::unordered_map<int, Node> closedList;

        openList.push(start);

        const std::vector<Action> actions = Action::getActions();

        while(!openList.empty()){
            Node current = openList.top();
            openList.pop();

            // if x&y position already visited, continue
            if (indexExistsInDict(closedList, current)){
                continue;
            }
            
            // else, add current to closed list and expand
            closedList[current.index_] = current;
            expand.push_back(current);
            
            // if current is the goal goal, return
            if(current == goal){
                path = convertClosedListToPath(closedList,start,goal);
                return true;
            }

            // else, iterate through actions
            for (const auto& action: actions){
                Node node_new = Node(current.x_ + action.get_x(), current.y_ + action.get_y(), current.g_ + action.get_cost());
                node_new.index_ = grid2Index(node_new.x_, node_new.y_);

                // if x&y position already visited, continue
                if(indexExistsInDict(closedList, node_new)){
                    continue;
                }

                node_new.parent_ = current.index_;

                if(isIllegalNode(global_costmap,node_new,current)){
                    continue;
                }
                
                openList.push(node_new);
            }
            
        }
        return false;
    }

    bool Dijkstra::indexExistsInDict(std::unordered_map<int, Node>& closedList, Node &n){
        return (closedList.find(n.index_) != closedList.end()) ? true : false;
    }

    bool Dijkstra::isIllegalNode(const unsigned char* global_costmap, Node &n, Node &current){
        return ((n.index_ < 0) || (n.index_ >= mapSize) || global_costmap[n.index_] >= lethal_cost_* factor_ && global_costmap[n.index_] >= global_costmap[current.index_]) ? true : false;
    }

    std::vector<Node> Dijkstra::convertClosedListToPath(std::unordered_map<int, Node>& closedList, const Node& start, 
                                                   const Node& goal){
        std::vector<Node> path;
        auto current = closedList.find(goal.index_);
        while(current->second != start){
            path.emplace_back(current->second.x_,current->second.y_);
            auto it = closedList.find(current->second.parent_);
            if(it != closedList.end()){
                current = it;
            }
            else{
                return {};
            }
        }
        path.push_back(start);
        return path;

    }

    bool Dijkstra::getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan){
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

    void Dijkstra::publishExpand(std::vector<Node>& expand){
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

    void Dijkstra::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
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

    bool Dijkstra::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;

        return true;
    }

    bool Dijkstra::world2Map(double worldX, double worldY, double& mapX, double& mapY){
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

    void Dijkstra::map2World(double mapX, double mapY, double& worldX, double& worldY){
        worldX = originX + (mapX + convert_offset) * resolution;
        worldY = originY + (mapY + convert_offset) * resolution;
    }

    void Dijkstra::map2Grid(double mapX, double mapY, int& gridX, int& gridY){
        gridX = (int)mapX;
        gridY = (int)mapY;
    }

    int Dijkstra::grid2Index(int x, int y){
        // ROS_INFO("Computeing grid2index.");
        return x + width * y;
    }

    void Dijkstra::outlineMap(unsigned char* costarr){
        
        unsigned char* pc = costarr;
        for (int i = 0; i < width; i++)
            *pc++ = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr + (height - 1) * width;
        for (int i = 0; i < width; i++)
            *pc++ = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr;
        for (int i = 0; i < height; i++, pc += width)
            *pc = costmap_2d::LETHAL_OBSTACLE;
        pc = costarr + width - 1;
        for (int i = 0; i < height; i++, pc += width)
            *pc = costmap_2d::LETHAL_OBSTACLE;
    }

 };

