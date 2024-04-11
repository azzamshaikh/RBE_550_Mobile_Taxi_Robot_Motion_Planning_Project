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
            // tBreak = 1+1/(mapSize); 
            // value = 0;
            
            // occupancyGridMap = new bool[mapSize];
            // for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++){
            //     for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++){
            //         unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

            //         if (cost == 0)
            //             occupancyGridMap[iy * width + ix] = true;
            //         else
            //             occupancyGridMap[iy * width + ix] = false;
            //     }
            // }

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan",1);

            expand_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree",1);

            make_plan_srv = private_nh.advertiseService("make_plan", &RRT::makePlanService, this);

            ROS_INFO("RRT Global Planner initialized successfully.");
            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing.");
        }
    }

    bool RRT::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan ){
        
        // MUTEX THREADS? 
        // start thread mutex
        boost::mutex::scoped_lock lock(mutex_);

        if (!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        plan.clear();

        ROS_INFO("RRT Global Planner");
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
        
        costmap_->setCost(gridGoalX, gridGoalY, costmap_2d::FREE_SPACE);

        // outline map????
        outlineMap(costmap_->getCharMap());

        // ROS_INFO("Creating path and expand vectors.");

        // calculate path
        std::vector<Node> path;
        std::vector<Node> expand;
        bool path_found = false;

        // ROS_INFO("Running RRT");
        // planning
        path_found = runRRT(costmap_->getCharMap(),start_node, goal_node, path, expand);

        if (path_found){
            // ROS_INFO("Getting path");
            if(getPlanFromPath(path, plan)){
                // ROS_INFO("Goal received");
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

    bool RRT::runRRT(const unsigned char* global_costmap,const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand){
        ROS_INFO("Inside RRT algorithm");
        path.clear();
        expand.clear();

        sampleList.clear();    
        costs_copy = global_costmap;

        start_copy = start;
        goal_copy = goal;

        sampleList.insert(std::make_pair(start.id_, start));
        expand.push_back(start);

        int iter = 0;
        while(iter < 3000){
            
            // ROS_INFO("inside while loop on iteration %i",iter);
            Node sample_node = sample(iter, goal);

            if(global_costmap[sample_node.id_] >= 253*0.5){
                continue;
            }

            if(sampleList.find(sample_node.id_) != sampleList.end()){
                continue;
            }

            Node new_node = findNearestNode(sampleList, sample_node);
            if(new_node.id_ == -1){
                continue;
            }
            else{
                sampleList.insert(std::make_pair(new_node.id_, new_node));
                expand.push_back(new_node);
            }

            if(goalFound(new_node)){
                // ROS_INFO("Goal found");
                path = convertClosedListToPath(sampleList, start, goal);
                // ROS_INFO("completed conversion");
                return true;
            }

            iter++;
        }
        return false;
    }

    Node RRT::sample(int & iter, const Node& goal){
        // ROS_INFO("INSIDE SAMPLING");
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<int> distr(0, mapSize - 1); // define the range
        const int id = distr(gen);
        int x, y;
        index2Grid(id, x, y);
        // if(iter % 100 == 0){
        //     return Node(goal.x_, goal.y_, 0, 0, goal.id_, 0);
        // }
        // else{
        //     return Node(x, y, 0, 0, id, 0);
        // }
        return Node(x, y, 0, 0, id, 0);
    }

    Node RRT::findNearestNode(std::unordered_map<int, Node> sampleList, const Node& node){
        // ROS_INFO("INSIDE FIND NEAREST NODE");
        Node nearest;
        Node new_node(node);

        double min_dist = std::numeric_limits<double>::max();

        for(const auto & node: sampleList){
            double dist = math::euclidean_distance(node.second, new_node);

            if(dist < min_dist){
                nearest = node.second;
                new_node.parentID_ = nearest.id_;
                new_node.g_ = dist + node.second.g_;
                min_dist = dist;
            }
        }

        if (min_dist > 15){
            double theta = math::angle(nearest,new_node);
            new_node.x_ = nearest.x_ +(int)(15 *cos(theta));
            new_node.y_ = nearest.y_ +(int)(15 *sin(theta));
            new_node.id_ = grid2Index(new_node.x_, new_node.y_);
            new_node.g_ = 15 + nearest.g_;
        }

        if (collisionCheck(new_node, nearest)){
            new_node.id_ = -1;
        }

        return new_node;


    }

    bool RRT::collisionCheck(const Node& n1, const Node& n2){
        double theta = math::angle(n1, n2);
        double dist = math::euclidean_distance(n1, n2);

        if(dist > 15){
            return true;
        }

        int step = (int)(dist/resolution);
        for(int i = 0; i < step; i++){
            float line_x = (float)n1.x_ + (float)(i*resolution*cos(theta));
            float line_y = (float)n1.y_ + (float)(i*resolution*cos(theta));
            if(costs_copy[grid2Index((int)line_x, (int)line_y)] >= 253*0.5){
                return true;
            }
        }
        return false;

    }

    bool RRT::goalFound(const Node& node){
        auto dist = math::euclidean_distance(node, goal_copy);
        if (dist > 15){
            return false;
        }

        if(!collisionCheck(node, goal_copy)){
            Node new_goal(goal_copy.x_, goal_copy.y_, dist+node.g_, 0, grid2Index(goal_copy.x_, goal_copy.y_), node.id_);
            sampleList.insert(std::make_pair(new_goal.id_, new_goal));
            return true;
        }
        return false;
    }

    std::vector<Node> RRT::convertClosedListToPath(std::unordered_map<int, Node>& closedList, const Node& start, 
                                                   const Node& goal){
        ROS_INFO("Converting closed list to path");                                 
        std::vector<Node> path;
        auto current = closedList.find(goal.id_);
        while(current->second != start){
            path.emplace_back(current->second.x_,current->second.y_);
            auto it = closedList.find(current->second.parentID_);
            if(it != closedList.end()){
                current = it;
            }
            else{
                return {};
            }
        }
        path.push_back(start);
        ROS_INFO("returning path");
        return path;

    }


    bool RRT::getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan){
        ROS_INFO("Getting plan from path");
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
            if (node.parentID_ != 0){
                tree.header.stamp = ros::Time::now();

                geometry_msgs::Point point1, point2;
                std_msgs::ColorRGBA color1, color2;
                int point1x, point1y, point2x, point2y;

                index2Grid(node.id_, point1x, point1y);
                map2World(point1x, point1y, point1.x, point1.y);
                point1.x = (point1.x + convert_offset) + 0;//costmap_->getOriginX();
                point1.y = (point1.y + convert_offset) + 0;//costmap_->getOriginY();
                point1.z = 1.0;

                index2Grid(node.parentID_, point2x, point2y);
                map2World(point2x, point2y, point2.x, point2.y);
                point2.x = (point2.x + convert_offset) + 0;//costmap_->getOriginX();
                point2.y = (point2.y + convert_offset) + 0;//costmap_->getOriginY();
                point2.z = 1.0;

                color1.r = 0.43;
                color2.r = 0.43;

                color1.g = 0.54;
                color2.g = 0.54;

                color1.b = 0.24;
                color2.b = 0.24;

                color1.a = 0.5;
                color2.a = 0.5;

                tree.points.push_back(point1);
                tree.points.push_back(point2);
                tree.colors.push_back(color1);
                tree.colors.push_back(color2);

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

        mapX = (worldX-originX) / resolution - convert_offset; // double check this convert shit
        mapY = (worldY-originY) / resolution - convert_offset; // double check this convert shit
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
        // ROS_INFO("Computeing grid2index.");
        return x + width * y;
    }

    void RRT::index2Grid(int i, int& x, int& y){
        x = i % width;
        y = i / width;
    }


    void RRT::outlineMap(unsigned char* costarr){
        
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

