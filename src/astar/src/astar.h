/** include the libraries you need in your planner here */
 /** for global path planner interface */
 #include <ros/ros.h>
 #include <costmap_2d/costmap_2d_ros.h>
 #include <costmap_2d/costmap_2d.h>
 #include <nav_core/base_global_planner.h>
 #include <geometry_msgs/PoseStamped.h>
 #include <angles/angles.h>
 #include <base_local_planner/world_model.h>
 #include <base_local_planner/costmap_model.h>
 #include <vector>
 #include <tf/tf.h>
 #include <set>

 using std::string;
 using std::vector;

 #ifndef ASTAR_CPP
 #define ASTAR_CPP

 struct Node {
    int currentCell;
    float f_cost;
 };

 namespace A_Star_Planner {

 class AStar : public nav_core::BaseGlobalPlanner {
    public:

        AStar();
        AStar(ros::NodeHandle &);
        AStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        ros::NodeHandle ROSNodeHandle;

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal,
                    std::vector<geometry_msgs::PoseStamped>& plan
                    );
        
        
        bool initialized_;
        costmap_2d::Costmap2DROS* costmap_ros_;
        costmap_2d::Costmap2D* costmap_;

        float originX;
        float originY;
        float resolution;
        int width;
        int height;



        // methods
        vector<int> runAStar(int startCell,int goalCell);
        vector<int> findPath(int startCell,int goalCell,float g_score[]);
        vector<int> constructPath(int startCell, int goalCell, float g_score[]);

        void getMapCoordinates(float& x, float& y);
        int convertToCellIndex(float x, float y);
        void convertToCoordinate(int index, float& x, float& y);
        bool isCellInBounds(float x, float y);
        bool isStartAndGoalValid(int startCell, int goalCell);
        vector<int> findFreeNeighborCells(int cellIndex);
        void addNeighbor(std::multiset<Node> &openList, int neighborCell, int goalCell, float g_score[]);
        float calculatedHCost(int startCell,int goalCell);
        float getMoveCost(int cell1, int cell2);
        float getMoveCost(int i1, int j1, int i2, int j2);


        int getCellIndex(int i, int j);
        int getCellRowID(int index);
        int getCellColID(int index);
        bool isFree(int cellIndex);
        bool isFree(int i, int j);

    };
        
 };
 #endif