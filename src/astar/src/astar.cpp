#include <pluginlib/class_list_macros.h>
#include "astar.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(A_Star_Planner::AStar, nav_core::BaseGlobalPlanner)

using namespace std;


int value;
int mapSize;
float tBreak; // coefficient for breaking ties
bool* occupancyGridMap;
float infinity = std::numeric_limits<float>::infinity();


//Default Constructor
namespace A_Star_Planner {

    // Default constructor
    AStar::AStar(){}

    // Constructor with node handle
    AStar::AStar(ros::NodeHandle &nh){
        ROSNodeHandle = nh;
    }

    // Constructor with cost map
    AStar::AStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
    }

    void AStar::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        if(!initialized_){
            costmap_ros_ = costmap_ros; // Initialize the costmap_ros_ attribute to the parameter
            
            costmap_ = costmap_ros_->getCostmap();   // Get the costmap_ from costmap_ros_

            // Initialize other planner parameters
            ros::NodeHandle private_nh("~/" + name);
            // private_nh.param("step_size", step_size_, costmap->getResolution());
            // private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);

            originX = costmap_->getOriginX();
            originY = costmap_->getOriginY();

            width = costmap_->getSizeInCellsX();
            height = costmap_->getSizeInCellsY();
            resolution = costmap_->getResolution();

            mapSize = width*height;
            tBreak = 1+1/(mapSize); 
            value = 0;
            
            occupancyGridMap = new bool[mapSize];
            for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++){
                for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++){
                    unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                    if (cost == 0)
                        occupancyGridMap[iy * width + ix] = true;
                    else
                        occupancyGridMap[iy * width + ix] = false;
                }
            }

            ROS_INFO("A* Global Planner initialized successfully.");
            initialized_ = true;
        }
        else{
            ROS_WARN("This planner has already been initialized... doing nothing.");
        }
    }

    bool AStar::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                        std::vector<geometry_msgs::PoseStamped>& plan ){
        
        if (!initialized_){
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y,
                    goal.pose.position.x, goal.pose.position.y);

        plan.clear();

        ROS_INFO("A* Global Planner");
        ROS_INFO("Current Position: ( %.2lf, %.2lf)", start.pose.position.x, start.pose.position.y);
        ROS_INFO("Goal Position: ( %.2lf, %.2lf)", goal.pose.position.x, goal.pose.position.y);

        if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.",
                    costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        // Convert the start and goal positions

        float startX = start.pose.position.x;
        float startY = start.pose.position.y;

        float goalX = goal.pose.position.x;
        float goalY = goal.pose.position.y;

        getMapCoordinates(startX,startY);
        getMapCoordinates(goalX,goalY);

        int startCell;
        int goalCell;

        if(isCellInBounds(startX,startY) && isCellInBounds(goalX,goalY)){
            startCell = convertToCellIndex(startX,startY);
            goalCell = convertToCellIndex(goalX,goalY);
        }
        else{
            ROS_WARN("The start or goal is out of the map");
            return false;
        }

        // Call global planner

        if(isStartAndGoalValid(startCell,goalCell)){
            vector<int> bestPath;
            bestPath.clear();

            bestPath = runAStar(startCell,goalCell);

            // If the global planner finds a path
            if(bestPath.size() > 0){
                // Convert the path

                for(int i = 0; i < bestPath.size(); i++){
                    float x = 0.0;
                    float y = 0.0;

                    float prev_x = 0.0;
                    float prev_y = 0.0;

                    int index = bestPath[i];
                    int prev_index;

                    convertToCoordinate(index,x,y);

                    if(i != 0){
                        prev_index = bestPath[i-1];
                    }
                    else{
                        prev_index = index;
                    }

                    convertToCoordinate(prev_index,prev_x,prev_y);

                    // Get angle between current and previous index to orient the robot
                    tf::Vector3 vectToTarget;
                    vectToTarget.setValue(x-prev_x,y-prev_y,0.0);
                    float theta = atan2((double)vectToTarget.y(), (double)vectToTarget.x());

                    geometry_msgs::PoseStamped pose = goal;

                    pose.pose.position.x = x;
                    pose.pose.position.y = y;
                    pose.pose.position.z = 0.0;

                    pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

                    plan.push_back(pose);
                }
                return true;
            }
            else{
                ROS_WARN("The planner has failed to find a path. Please find another goal position.");
                return false;
            }
        }
        else{
            ROS_WARN("The start or goal cell is not valid.");
            return false;
        }
    }

    vector<int> AStar::runAStar(int startCell, int goalCell){
        vector<int> bestPath;

        float g_score[mapSize];

        for (uint i = 0; i < mapSize; i++){
            g_score[i] = infinity;
        }

        bestPath = findPath(startCell,goalCell,g_score);

        return bestPath;
    }

    vector<int> AStar::findPath(int startCell, int goalCell, float g_score[]){
        value++;
        vector<int> bestPath;
        vector<int> emptyPath;
        Node node;

        multiset<Node> openNodeList;
        int currentCell;

        // Calculate g_score and f_score of the start position
        g_score[startCell]=0;
        node.currentCell = startCell;
        node.f_cost = g_score[startCell] + calculatedHCost(startCell,goalCell);

        // Add the start node to the open list
        openNodeList.insert(node);
        currentCell = startCell;

        // While the open list is not empty and g score at the goal is infinity, continue the search
        while(!openNodeList.empty() && g_score[goalCell]==infinity){
            // Choose the grid cell that has the lowest f_cost in the open set
            currentCell = openNodeList.begin()->currentCell;

            // Delete that cell from the open list
            openNodeList.erase(openNodeList.begin());

            // Search the neighbors of the current cell
            vector<int> neighborCells;
            neighborCells = findFreeNeighborCells(currentCell);

            // Loop through each neighbor
            for(uint i=0; i<neighborCells.size(); i++){
                // If the g_score of the neighbor is INF, it means its not visited
                if(g_score[neighborCells[i]]==infinity){
                    g_score[neighborCells[i]]=g_score[currentCell]+getMoveCost(currentCell,neighborCells[i]);
                    addNeighbor(openNodeList, neighborCells[i],goalCell,g_score);
                }
            }
        }

        // If goal cell is no longer at a cost of infinity, construct a path
        if(g_score[goalCell] != infinity){
            bestPath = constructPath(startCell,goalCell,g_score);
            return bestPath;
        }
        else{
            ROS_WARN("Failed to find a path!");
            return emptyPath;
        }

    }

    vector<int> AStar::constructPath(int startCell, int goalCell, float g_score[]){
        vector<int> bestPath;
        vector<int> path;

        path.insert(path.begin()+bestPath.size(), goalCell);
        int currentCell = goalCell;

        while(currentCell != startCell){
            vector<int> neighborCells;
            neighborCells = findFreeNeighborCells(currentCell);

            vector<float> gScoreNeighbors;
            for(uint i=0; i<neighborCells.size(); i++){
                gScoreNeighbors.push_back(g_score[neighborCells[i]]);
            }

            int posMinGSore = distance(gScoreNeighbors.begin(),min_element(gScoreNeighbors.begin(),gScoreNeighbors.end()));
            currentCell = neighborCells[posMinGSore];

            // Insert the neighbor in the path
            path.insert(path.begin()+path.size(), currentCell);
        }
        for(uint i=0; i<path.size();i++){
            bestPath.insert(bestPath.begin()+bestPath.size(),path[path.size()-(i+1)]);
        }
        return bestPath;
    }

    float AStar::calculatedHCost(int startCell,int goalCell){
        int x1 = getCellRowID(goalCell);
        int y1 = getCellColID(goalCell);
        int x2 = getCellRowID(startCell);
        int y2 = getCellColID(startCell);
        return abs(x1-x2) + abs(y1-y2);
    }

    float AStar::getMoveCost(int cell1, int cell2){
        int i1 = 0, j1 =0, i2=0, j2 =0;
        
        i1 = getCellRowID(cell1);
        j1 = getCellColID(cell1);
        i2 = getCellRowID(cell2);
        j2 = getCellColID(cell2);

        return getMoveCost(i1,j1,i2,j2);
    }

    float AStar::getMoveCost(int i1, int j1, int i2, int j2){
        
        // Start cost with max value. Change it to a real cost of cells if they are connected
        float moveCost = infinity;

        // Check diagonal movements
        if((j2==j1+1 && i2==i1+1) || (j2==j1+1 && i2==i1-1) || (j2==j1-1 && i2==i1+1) || (j2==j1-1 && i2==i1-1)){
            moveCost = 1.4;
        }
        // Check translational movements
        else{
            if((j2==j1 && i2==i1+1) || (j2==j1 && i2==i1-1) || (j2==j1+1 && i2==i1) || (j2==j1-1 && i2==i1)){
                moveCost = 1;
            }
        }
        return moveCost;
    }

    void AStar::getMapCoordinates(float& x, float& y){
        x = x - originX;
        y = y - originY;
    }

    int AStar::convertToCellIndex(float x, float y){
        int cellIndex;

        float newX = x/resolution;
        float newY = y/resolution;

        cellIndex = getCellIndex(newY,newX);

        return cellIndex;
    }

    void AStar::convertToCoordinate(int index, float& x, float& y){
        x = getCellColID(index) * resolution;
        y = getCellRowID(index) * resolution;

        x = x + originX;
        y = y + originY;
    }

    bool AStar::isCellInBounds(float x, float y){
        bool valid = true;

        if (x > (width * resolution) || y > (height * resolution))
            valid = false;
        
        return valid;
    }

    bool AStar::isStartAndGoalValid(int startCell, int goalCell){
        bool isValid = true;
        bool isFreeStartCell = isFree(startCell);
        bool isFreeGoalCell = isFree(goalCell);

        if (startCell == goalCell){
            isValid = false;
        }
        else{
            if (!isFreeStartCell && !isFreeGoalCell){
                isValid = false;
            }
            else{
                if(!isFreeStartCell){
                    isValid = false;
                }
                else{
                    if(!isFreeGoalCell){
                        isValid = false;
                    }
                    else{
                        if(findFreeNeighborCells(goalCell).size() == 0){
                            isValid = false;
                        }
                        else{
                            if(findFreeNeighborCells(startCell).size() == 0){
                                isValid = false;
                            }
                        }
                    }
                }
            }
        }
        return isValid;
    }

    vector<int> AStar::findFreeNeighborCells(int cellIndex){
        int rowIndex = getCellRowID(cellIndex);
        int colIndex = getCellColID(cellIndex);
        int neighborIndex;

        vector<int> freeNeighborCells;

        for(int i = -1;i<=1;i++){
            for(int j = -1;j<=1;j++){
                // Check whether the index is valid
                if((rowIndex+i >= 0) && (rowIndex+1 < height) && (colIndex+j >= 0) && (colIndex+j < width) && (!(i==0 && j==0))){
                    neighborIndex = getCellIndex(rowIndex+i,colIndex+j);
                    if(isFree(neighborIndex)){
                        freeNeighborCells.push_back(neighborIndex);
                    }
                }
            }
        }
        return freeNeighborCells;
    }

    void AStar::addNeighbor(std::multiset<Node> &openList, int neighborCell, int goalCell, float g_score[]){
        Node node;
        node.currentCell = neighborCell;
        node.f_cost = g_score[neighborCell] + calculatedHCost(neighborCell,goalCell);
        openList.insert(node);
    }


    int AStar::getCellIndex(int i, int j){
        // Get the index of the cell to be used in the path
        return (i*width)+j;
    }

    int AStar::getCellRowID(int index){
        // Get the row ID from cell index
        return index/width;
    }

    int AStar::getCellColID(int index){
        // Get the column ID from cell index
        return index%width;
    }

    bool AStar::isFree(int cellIndex){
        return occupancyGridMap[cellIndex];
    }
    
    bool AStar::isFree(int i, int j){
        return occupancyGridMap[getCellIndex(i,j)];
    }
 };

 bool operator<(Node const &n1, Node const &n2) {return n1.f_cost < n2.f_cost;}