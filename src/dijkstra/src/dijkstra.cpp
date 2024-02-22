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


        // // Call global planner

        // if(isStartAndGoalValid(startCell,goalCell)){
        //     vector<int> bestPath;
        //     bestPath.clear();

        //     bestPath = runAStar(startCell,goalCell);

        //     // If the global planner finds a path
        //     if(bestPath.size() > 0){
        //         // Convert the path

        //         for(int i = 0; i < bestPath.size(); i++){
        //             float x = 0.0;
        //             float y = 0.0;

        //             float prev_x = 0.0;
        //             float prev_y = 0.0;

        //             int index = bestPath[i];
        //             int prev_index;

        //             convertToCoordinate(index,x,y);

        //             if(i != 0){
        //                 prev_index = bestPath[i-1];
        //             }
        //             else{
        //                 prev_index = index;
        //             }

        //             convertToCoordinate(prev_index,prev_x,prev_y);

        //             // Get angle between current and previous index to orient the robot
        //             tf::Vector3 vectToTarget;
        //             vectToTarget.setValue(x-prev_x,y-prev_y,0.0);
        //             float theta = atan2((double)vectToTarget.y(), (double)vectToTarget.x());

        //             geometry_msgs::PoseStamped pose = goal;

        //             pose.pose.position.x = x;
        //             pose.pose.position.y = y;
        //             pose.pose.position.z = 0.0;

        //             pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        //             plan.push_back(pose);
        //         }
        //         return true;
        //     }
        //     else{
        //         ROS_WARN("The planner has failed to find a path. Please find another goal position.");
        //         return false;
        //     }
        // }
        // else{
        //     ROS_WARN("The start or goal cell is not valid.");
        //     return false;
        // }
    }

    bool Dijkstra::runDijkstra(const unsigned char* global_costmap,const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand){
        path.clear();
        expand.clear();

        //open list and closed list
        std::priority_queue<Node, std::vector<Node>, Node::compare_cost> openList;
        std::unordered_map<int, Node> closedList;

        openList.push(start);

        const std::vector<Node> motions = Node::getMotion();

        while(!openList.empty()){
            Node current = openList.top();
            openList.pop();

            if (closedList.find(current.id_) != closedList.end()){
                continue;
            }

            closedList.insert(std::make_pair(current.id_,current));
            expand.push_back(current);

            if(current == goal){
                path = convertClosedListToPath(closedList,start,goal);
                return true;
            }

            for (const auto& motion: motions){
                Node node_new = current + motion;
                node_new.id_ = grid2Index(node_new.x_, node_new.y_);

                if(closedList.find(node_new.id_) != closedList.end()){
                    continue;
                }

                node_new.parentID_ = current.id_;

                if((node_new.id_ < 0) || (node_new.id_ >= mapSize) || 
                   (global_costmap[node_new.id_] >= lethal_cost_ * factor_ &&
                    global_costmap[node_new.id_] >= global_costmap[current.id_])){
                        continue;
                    }
                
                openList.push(node_new);
            }
            
        }
        return false;
    }

    std::vector<Node> Dijkstra::convertClosedListToPath(std::unordered_map<int, Node>& closedList, const Node& start, 
                                                   const Node& goal){
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
        return path;

    }

    // vector<int> AStar::runAStar(int startCell, int goalCell){
    //     vector<int> bestPath;

    //     float g_score[mapSize];

    //     for (uint i = 0; i < mapSize; i++){
    //         g_score[i] = infinity;
    //     }

    //     bestPath = findPath(startCell,goalCell,g_score);

    //     return bestPath;
    // }

    // vector<int> AStar::findPath(int startCell, int goalCell, float g_score[]){
    //     value++;
    //     vector<int> bestPath;
    //     vector<int> emptyPath;
    //     Node node;

    //     multiset<Node> openNodeList;
    //     int currentCell;

    //     // Calculate g_score and f_score of the start position
    //     g_score[startCell]=0;
    //     node.currentCell = startCell;
    //     node.f_cost = g_score[startCell] + calculatedHCost(startCell,goalCell);

    //     // Add the start node to the open list
    //     openNodeList.insert(node);
    //     currentCell = startCell;

    //     // While the open list is not empty and g score at the goal is infinity, continue the search
    //     while(!openNodeList.empty() && g_score[goalCell]==infinity){
    //         // Choose the grid cell that has the lowest f_cost in the open set
    //         currentCell = openNodeList.begin()->currentCell;

    //         // Delete that cell from the open list
    //         openNodeList.erase(openNodeList.begin());

    //         // Search the neighbors of the current cell
    //         vector<int> neighborCells;
    //         neighborCells = findFreeNeighborCells(currentCell);

    //         // Loop through each neighbor
    //         for(uint i=0; i<neighborCells.size(); i++){
    //             // If the g_score of the neighbor is INF, it means its not visited
    //             if(g_score[neighborCells[i]]==infinity){
    //                 g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
    //                 addNeighbor(openNodeList, neighborCells[i],goalCell,g_score);
    //             }
    //         }
    //     }

    //     // If goal cell is no longer at a cost of infinity, construct a path
    //     if(g_score[goalCell] != infinity){
    //         bestPath = constructPath(startCell,goalCell,g_score);
    //         return bestPath;
    //     }
    //     else{
    //         ROS_WARN("Failed to find a path!");
    //         return emptyPath;
    //     }

    // }

    // vector<int> AStar::constructPath(int startCell, int goalCell, float g_score[]){
    //     vector<int> bestPath;
    //     vector<int> path;

    //     path.insert(path.begin()+bestPath.size(), goalCell);
    //     int currentCell = goalCell;

    //     while(currentCell != startCell){
    //         vector<int> neighborCells;
    //         neighborCells = findFreeNeighborCells(currentCell);

    //         vector<float> gScoreNeighbors;
    //         for(uint i=0; i<neighborCells.size(); i++){
    //             gScoreNeighbors.push_back(g_score[neighborCells[i]]);
    //         }

    //         int posMinGSore = distance(gScoreNeighbors.begin(),min_element(gScoreNeighbors.begin(),gScoreNeighbors.end()));
    //         currentCell = neighborCells[posMinGSore];

    //         // Insert the neighbor in the path
    //         path.insert(path.begin()+path.size(), currentCell);
    //     }
    //     for(uint i=0; i<path.size();i++){
    //         bestPath.insert(bestPath.begin()+bestPath.size(),path[path.size()-(i+1)]);
    //     }
    //     return bestPath;
    // }

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
            grid.data[expand[i].id_] = 50;
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

        mapX = (worldX-originX) / resolution - convert_offset; // double check this convert shit
        mapY = (worldY-originY) / resolution - convert_offset; // double check this convert shit
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
    
    // float AStar::calculatedHCost(int startCell,int goalCell){
    //     int x1 = getCellRowID(goalCell);
    //     int y1 = getCellColID(goalCell);
    //     int x2 = getCellRowID(startCell);
    //     int y2 = getCellColID(startCell);
    //     return abs(x1-x2) + abs(y1-y2);
    // }

    // float AStar::getMoveCost(int cell1, int cell2){
    //     int i1 = 0, j1 =0, i2=0, j2 =0;
        
    //     i1 = getCellRowID(cell1);
    //     j1 = getCellColID(cell1);
    //     i2 = getCellRowID(cell2);
    //     j2 = getCellColID(cell2);

    //     return getMoveCost(i1,j1,i2,j2);
    // }

    // float AStar::getMoveCost(int i1, int j1, int i2, int j2){
        
    //     // Start cost with max value. Change it to a real cost of cells if they are connected
    //     float moveCost = infinity;

    //     // Check diagonal movements
    //     if((j2==j1+1 && i2==i1+1) || (j2==j1+1 && i2==i1-1) || (j2==j1-1 && i2==i1+1) || (j2==j1-1 && i2==i1-1)){
    //         moveCost = 1.4;
    //     }
    //     // Check translational movements
    //     else{
    //         if((j2==j1 && i2==i1+1) || (j2==j1 && i2==i1-1) || (j2==j1+1 && i2==i1) || (j2==j1-1 && i2==i1)){
    //             moveCost = 1;
    //         }
    //     }
    //     return moveCost;
    // }

    // void AStar::getMapCoordinates(double& x, double& y){
    //     x = x - originX;
    //     y = y - originY;
    // }

    // int AStar::convertToCellIndex(float x, float y){
    //     int cellIndex;

    //     float newX = x/resolution;
    //     float newY = y/resolution;

    //     cellIndex = getCellIndex(newY,newX);

    //     return cellIndex;
    // }

    // void AStar::convertToCoordinate(int index, float& x, float& y){
    //     x = getCellColID(index) * resolution;
    //     y = getCellRowID(index) * resolution;

    //     x = x + originX;
    //     y = y + originY;
    // }

    // bool AStar::isCellInBounds(float x, float y){
    //     bool valid = true;

    //     if (x > (width * resolution) || y > (height * resolution))
    //         valid = false;
        
    //     return valid;
    // }

    // bool AStar::isStartAndGoalValid(int startCell, int goalCell){
    //     bool isValid = true;
    //     bool isFreeStartCell = isFree(startCell);
    //     bool isFreeGoalCell = isFree(goalCell);

    //     if (startCell == goalCell){
    //         isValid = false;
    //     }
    //     else{
    //         if (!isFreeStartCell && !isFreeGoalCell){
    //             isValid = false;
    //         }
    //         else{
    //             if(!isFreeStartCell){
    //                 isValid = false;
    //             }
    //             else{
    //                 if(!isFreeGoalCell){
    //                     isValid = false;
    //                 }
    //                 else{
    //                     if(findFreeNeighborCells(goalCell).size() == 0){
    //                         isValid = false;
    //                     }
    //                     else{
    //                         if(findFreeNeighborCells(startCell).size() == 0){
    //                             isValid = false;
    //                         }
    //                     }
    //                 }
    //             }
    //         }
    //     }
    //     return isValid;
    // }

    // vector<int> AStar::findFreeNeighborCells(int cellIndex){
    //     int rowIndex = getCellRowID(cellIndex);
    //     int colIndex = getCellColID(cellIndex);
    //     int neighborIndex;

    //     vector<int> freeNeighborCells;

    //     for(int i = -1;i<=1;i++){
    //         for(int j = -1;j<=1;j++){
    //             // Check whether the index is valid
    //             if((rowIndex+i >= 0) && (rowIndex+1 < height) && (colIndex+j >= 0) && (colIndex+j < width) && (!(i==0 && j==0))){
    //                 neighborIndex = getCellIndex(rowIndex+i,colIndex+j);
    //                 if(isFree(neighborIndex)){
    //                     freeNeighborCells.push_back(neighborIndex);
    //                 }
    //             }
    //         }
    //     }
    //     return freeNeighborCells;
    // }

    // void AStar::addNeighbor(std::multiset<Node> &openList, int neighborCell, int goalCell, float g_score[]){
    //     Node node;
    //     node.currentCell = neighborCell;
    //     node.f_cost = g_score[neighborCell] + calculatedHCost(neighborCell,goalCell);
    //     openList.insert(node);
    // }


    // int AStar::getCellIndex(int i, int j){
    //     // Get the index of the cell to be used in the path
    //     // transforms from grid map (i,j) to grid index(i)
    //     return (i*width)+j;
    // }

    // int AStar::getCellRowID(int index){
    //     // Get the row ID from cell index
    //     return index/width;
    // }

    // int AStar::getCellColID(int index){
    //     // Get the column ID from cell index
    //     return index%width;
    // }

    // bool AStar::isFree(int cellIndex){
    //     return occupancyGridMap[cellIndex];
    // }
    
    // bool AStar::isFree(int i, int j){
    //     return occupancyGridMap[getCellIndex(i,j)];
    // }

    
 };

