#ifndef ASTAR_HPP
#define ASTAR_HPP

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
#include <nav_msgs/GetPlan.h>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using std::string;
using std::vector;

namespace A_Star_Planner {

   class Node{
      public:
         Node(int x=0, int y=0, double g=0.0, double h=0.0, double f=0.0, int index=0, int parent= 0){
               x_ = x;
               y_ = y;
               g_ = g;
               h_ = h;
               f_ = f;
               index_ = index;
               parent_ = parent;
         }

         void distance(const Node& goal){
               h_ = std::hypot(x_ - goal.x_, y_ - goal.y_);
         }

         void updateHeuristic(){
               f_ = g_ + h_;
         }

         int x_, y_, index_, parent_;
         double g_, h_, f_;
   };

   class AStar : public nav_core::BaseGlobalPlanner {
      public:

         AStar();
         AStar(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

         /** overridden classes from interface nav_core::BaseGlobalPlanner **/
         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
         bool makePlan(const geometry_msgs::PoseStamped& start,
                     const geometry_msgs::PoseStamped& goal,
                     std::vector<geometry_msgs::PoseStamped>& plan
                     );
         

         // methods
         bool runAStar(const unsigned char* global_costmap,const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);
         bool goalCheck(Node &node, const Node &goal);
         bool indexExistsInDict(std::unordered_map<int, Node> &closedList, Node &n);
         bool isIllegalNode(const unsigned char *global_costmap, Node &n, Node &current);
         vector<int> findPath(int startCell, int goalCell, float g_score[]);
         vector<int> constructPath(int startCell, int goalCell, float g_score[]);

         
         bool world2Map(double worldX, double Worldy, double& mapX, double& mapY);
         void map2World(double mapX, double mapY, double& worldX, double& worldY);
         void map2Grid(double mapX, double mapY, int& gridX, int& gridY);
         int grid2Index(int x, int y);

         void outlineMap(unsigned char* costarr);

         void publishExpand(std::vector<Node>& expand);
         void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
         bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
         bool getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan);
         
      private:

         bool initialized_;
         costmap_2d::Costmap2DROS* costmap_ros_;
         costmap_2d::Costmap2D* costmap_;
         string frame_id_; //costmap frame id

         double originX;   // costmap origin
         double originY;   // costmap origin
         double resolution;   // costmap resolution
         double convert_offset;  // offset of transform from world to grid map
         

         int width;     // costmap size
         int height;    // costmap size
         int mapSize;   // pixel total

         ros::Publisher plan_pub_;  // path planning publisher
         ros::Publisher expand_pub_; // node explorer publisher
         ros::ServiceServer make_plan_srv; // planning service

         unsigned char lethal_cost_ = 253, neurtral_cost_ = 50;
         double factor_ = 0.25;




   };

   class Action{
      private:
         int x_;
         int y_;
         double cost_;
      
      public:
         Action(int x = 0, int y = 0, double cost = 0.0){
            x_ = x;
            y_ = y;
            cost_ = cost;
         }

         int get_x() const{
            return x_;
         }
        
         int get_y() const{
            return y_;
         }
        
         double get_cost() const{
            return cost_;
         }

         static vector<Action> getActions(){
            return {
               Action(0, 1, 1),
               Action(1, 0, 1),
               Action(0, -1, 1),
               Action(-1, 0, 1),
               Action(1, 1, std::sqrt(2)),
               Action(1, -1, std::sqrt(2)),
               Action(-1, 1, std::sqrt(2)),
               Action(-1, -1, std::sqrt(2)),
            };
         }

   };
      
};
#endif