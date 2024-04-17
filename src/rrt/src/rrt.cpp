#include "rrt.hpp"
#include <pluginlib/class_list_macros.h>
#include <limits>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(RRT_Planner::RRT, nav_core::BaseGlobalPlanner)

using namespace std;


//Default Constructor
namespace RRT_Planner {

    // Default constructor
    RRT::RRT() : initialized_(false), costmap_(nullptr) {}

    // Constructor with cost map
    RRT::RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void RRT::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros; // Initialize the costmap_ros_ attribute to the parameter
            costmap_ = costmap_ros_->getCostmap();   // Get the costmap_ from costmap_ros_
            frame_id_ = costmap_ros_->getGlobalFrameID();
            
            // Initialize other planner parameters
            ros::NodeHandle private_nh("~/" + name);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);

            expand_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree",1);

            make_plan_srv = private_nh.advertiseService("make_plan", &RRT::makePlanService, this);

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();
            convert_offset = 0.0;

            mapSize = width*height;

            ROS_INFO("RRT Global Planner initialized successfully.");
            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing.");
        }
    }

    bool RRT::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan ){
        
        if (!initialized_){
            ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
            return false;
        }

        plan.clear();

        ROS_INFO("RRT Global Planner");
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
        double mapStartX, mapStartY, mapGoalX, mapGoalY;
        if(!world2Map(worldX,worldY,mapStartX, mapStartY)){
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
        // ROS_INFO("Converting from map to grid values.");
        int gridStartX, gridStartY, gridGoalX, gridGoalY;
        map2Grid(mapStartX,mapStartY,gridStartX, gridStartY);
        map2Grid(mapGoalX, mapGoalY,gridGoalX, gridGoalY);


        // ROS_INFO("Creating start and goal nodes.");

        Node start_node(gridStartX,gridStartY,0,grid2Index(gridStartX,gridStartY),0);
        Node goal_node(gridGoalX,gridGoalY,0,grid2Index(gridGoalX,gridGoalY),0);
        
        costmap_->setCost(gridGoalX, gridGoalY, costmap_2d::FREE_SPACE);

        // ROS_INFO("Creating path and expand vectors.");

        // calculate path
        std::vector<Node> path, expand;
        bool found_legal = false;

        // ROS_INFO("Running RRT");
        // planning
        found_legal = runRRT(costmap_->getCharMap(),start_node, goal_node, path, expand);

        if (found_legal){
            // ROS_INFO("Getting path");
            if(getPlanFromPath(path, plan)){
                // ROS_INFO("Goal received");
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

    bool RRT::runRRT(const unsigned char* global_costmap,const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand){
        path.clear();
        expand.clear();
        sampleList.clear();    

        start_copy = start;
        goal_copy = goal;

        //sampleList.insert(std::make_pair(start.id_, start));
        sampleList[start.index_] = start;
        expand.push_back(start);

        int iter = 0;
        while(iter < 3000){
            
            // ROS_INFO("inside while loop on iteration %i",iter);
            Node sample_node = sample();

            if(isIllegalSample(sample_node,global_costmap) || indexExistsInDict(sampleList,sample_node)){
                continue;
            }

            Node new_node = findNearestNode(sampleList, sample_node, global_costmap);
            if(new_node.index_ == -1){
                continue;
            }
            else{
                sampleList[new_node.index_] = new_node;
                expand.push_back(new_node);
            }

            if(goalFound(new_node, goal, global_costmap)){
                // ROS_INFO("Goal found");
                auto current = sampleList.find(goal.index_);
                while(!goalCheck(current->second, start)){
                    path.emplace_back(current->second.x_,current->second.y_);
                    int key = current->second.parent_; // get value from dict and get its parent
                    Node next = sampleList.at(key);
                    if(indexExistsInDict(sampleList, next)){
                        current = sampleList.find(key);
                    }
                    else{
                        return {};
                    }
                }
                path.push_back(start);

                return true;
            }

            iter++;
        }
        return false;
    }

    bool RRT::goalCheck(Node& node, const Node& goal){
        return (node.x_ == goal.x_ && node.y_ == goal.y_) ? true : false;
    }

    Node RRT::sample(){
        // ROS_INFO("INSIDE SAMPLING");
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<int> distr(0, mapSize - 1); // define the range
        std::uniform_int_distribution<int> sendGoal(0,10);
        const int id = distr(gen);
        int x, y;
        index2Grid(id, x, y);
        return Node(x, y, 0, id, 0);
    }

    bool RRT::isIllegalSample(const Node& sample, const unsigned char* global_costmap){
        return (global_costmap[sample.index_] >= 253*0.5) ? true : false;
    }

    Node RRT::findNearestNode(std::unordered_map<int, Node> sampleList, const Node& sampled_node, const unsigned char* global_costmap){
        // ROS_INFO("INSIDE FIND NEAREST NODE");
        Node nearest; // new node
        Node new_node(sampled_node); //copy of sampled node

        std::map<double, Node> distances;

        for(const auto & sample: sampleList){
            double dist = math::euclidean_distance(sample.second, new_node);
            distances[dist] = sample.second;
        }

        double dist = distances.begin()->first;

        nearest = sampleList.at(distances.begin()->second.index_);

        new_node.parent_ = nearest.index_;
        new_node.g_ = dist + nearest.g_;
        
        if (dist > 10){
            double theta = math::angle(nearest,new_node);
            new_node.x_ = (int)nearest.x_ + (int)(10 *cos(theta));
            new_node.y_ = (int)nearest.y_ + (int)(10 *sin(theta));
            new_node.index_= grid2Index(new_node.x_, new_node.y_);
            new_node.g_ = nearest.g_+10;
        }

        if (collisionCheck(new_node, nearest, global_costmap)){
            new_node.index_ = -1;
        }

        return new_node;


    }

    bool RRT::collisionCheck(const Node& n1, const Node& n2, const unsigned char* global_costmap){
        double theta = math::angle(n1, n2);
        double dist = math::euclidean_distance(n1, n2);

        if(dist > 10){
            return true;
        }

        int step = (int)(dist/resolution);
        for(int i = 0; i < step; i++){
            float x_step = (float)n1.x_ + (float)(i*resolution*cos(theta));
            float y_step = (float)n1.y_ + (float)(i*resolution*sin(theta));
            if(global_costmap[grid2Index((int)x_step, (int)y_step)] >= 253*0.5){
                return true;
            }
        }
        return false;

    }

    bool RRT::goalFound(const Node& node, const Node& goal, const unsigned char* global_costmap){
        auto dist = math::euclidean_distance(node, goal);
        if (dist > 10){
            return false;
        }

        if(!collisionCheck(node, goal, global_costmap)){
            Node new_goal(goal.x_, goal.y_, dist+node.g_, grid2Index(goal.x_, goal.y_), node.index_);
            sampleList[new_goal.index_] = new_goal;
            return true;
        }
        return false;
    }

    bool RRT::indexExistsInDict(std::unordered_map<int, Node>& list, Node &n){
        return (list.find(n.index_) != list.end()) ? true : false;
    }

    bool RRT::getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan){
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


    void RRT::publishExpand(std::vector<Node>& expand){
        ROS_DEBUG("Expand zone size: %1ld", expand.size());

        visualization_msgs::Marker tree;

        tree.header.frame_id = "map";
        tree.id = 0;
        tree.ns = "tree";
        tree.type = visualization_msgs::Marker::LINE_LIST;
        tree.action = visualization_msgs::Marker::ADD;
        tree.pose.orientation.w = 1.0;
        tree.scale.x = 0.05;

        for(auto node: expand){
            if (node.parent_ != 0){
                tree.header.stamp = ros::Time::now();

                geometry_msgs::Point point1, point2;
                std_msgs::ColorRGBA color;
                int point1x, point1y, point2x, point2y;

                index2Grid(node.index_, point1x, point1y);
                map2World(point1x, point1y, point1.x, point1.y);
                point1.z = 1.0;

                index2Grid(node.parent_, point2x, point2y);
                map2World(point2x, point2y, point2.x, point2.y);
                point2.z = 1.0;

                color.r = 1;
                color.g = 0;
                color.b = 0;
                color.a = 0.5;
                

                tree.points.push_back(point1);
                tree.points.push_back(point2);
                tree.colors.push_back(color);
                tree.colors.push_back(color);

                expand_pub_.publish(tree);
            }
        }
    }

    void RRT::publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan){
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

    bool RRT::makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp){
        makePlan(req.start, req.goal, resp.plan.poses);
        resp.plan.header.stamp = ros::Time::now();
        resp.plan.header.frame_id = frame_id_;

        return true;
    }

    bool RRT::world2Map(double worldX, double worldY, double& mapX, double& mapY){
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

    void RRT::map2World(double mapX, double mapY, double& worldX, double& worldY){
        worldX = originX + (mapX + convert_offset) * resolution;
        worldY = originY + (mapY + convert_offset) * resolution;
    }

    void RRT::map2Grid(double mapX, double mapY, int& gridX, int& gridY){
        gridX = (int)mapX;
        gridY = (int)mapY;
    }

    int RRT::grid2Index(int x, int y){        
        return x + width * y;
    }

    void RRT::index2Grid(int i, int& x, int& y){
        x = i % width;
        y = i / width;
    }
    
};

