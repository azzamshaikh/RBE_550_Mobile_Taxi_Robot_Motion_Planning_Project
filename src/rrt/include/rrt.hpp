#ifndef RRT_HPP
#define RRT_HPP

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
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <tf/tf.h>
#include <set>
#include <nav_msgs/GetPlan.h>
#include <random>

#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using std::string;
using std::vector;


namespace RRT_Planner {

   class Node{
    public:
        int x_, y_, index_, parent_;
        double g_;

        Node(int x=0, int y=0, double g=0.0, int index=0, int parent= 0){
            x_ = x;
            y_ = y;
            g_ = g;
            index_ = index;
            parent_ = parent;
        }
   };

   class RRT : public nav_core::BaseGlobalPlanner {
      public:

         RRT();
         RRT(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

         /** overridden classes from interface nav_core::BaseGlobalPlanner **/
         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
         bool makePlan(const geometry_msgs::PoseStamped& start,
                     const geometry_msgs::PoseStamped& goal,
                     std::vector<geometry_msgs::PoseStamped>& plan
                     );
         

         // methods
         // vector<int> runAStar(int startCell,int goalCell); original implemntation
         bool runRRT(const unsigned char* global_costmap,const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);
         vector<int> findPath(int startCell,int goalCell,float g_score[]);
         vector<int> constructPath(int startCell, int goalCell, float g_score[]);
         
         bool goalCheck(Node& node, const Node& goal);

         Node sample(Node& goal);
         bool isIllegalSample(const Node &sample, const unsigned char *global_costmap);
         Node findNearestNode(std::unordered_map<int, Node> sampleList, const Node &sampled_node, const unsigned char *global_costmap);
         bool collisionCheck(const Node& n1, const Node& n2,const unsigned char* global_costmap);
         bool goalFound(const Node& node, const Node& goal, const unsigned char* global_costmap);

         bool indexExistsInDict(std::unordered_map<int, Node> &list, Node &n);

         bool world2Map(double worldX, double Worldy, double& mapX, double& mapY);
         void map2World(double mapX, double mapY, double& worldX, double& worldY);
         void map2Grid(double mapX, double mapY, int& gridX, int& gridY);
         int grid2Index(int x, int y);
         void index2Grid(int i, int& x, int& y);

         void outlineMap(unsigned char* costarr);

         void publishExpand(std::vector<Node>& expand);
         void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
         bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
         bool getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan);
         std::vector<Node> convertClosedListToPath(std::unordered_map<int, Node>& closedList, const Node& start, 
                                                   const Node& goal);

         
      private:
         boost::mutex mutex_;
         bool initialized_;
         costmap_2d::Costmap2DROS* costmap_ros_;
         costmap_2d::Costmap2D* costmap_;
         string frame_id_; //costmap frame id

         double originX;   // costmap origin
         double originY;   // costmap origin
         double resolution;   // costmap resolution
         double convert_offset;  // offset of transform from world to grid map

         const unsigned char* costs_copy;
         std::unordered_map<int, Node> sampleList;
         Node start_copy;
         Node goal_copy;

         int width;     // costmap size
         int height;    // costmap size
         int mapSize;   // pixel total

         ros::Publisher plan_pub_;  // path planning publisher
         ros::Publisher expand_pub_; // node explorer publisher
         ros::ServiceServer make_plan_srv; // planning service

         unsigned char lethal_cost_ = 253;
         double factor_ = 0.25;

   };
      
};

namespace math {
    double euclidean_distance(const RRT_Planner::Node& n1, const RRT_Planner::Node& n2){
        return std::hypot(n1.x_ - n2.x_, n1.y_ - n2.y_);
    }
    double angle(const RRT_Planner::Node& n1, const RRT_Planner::Node& n2){
        return atan2(n2.y_ - n1.y_, n2.x_ - n1.x_);
    }
};


#endif