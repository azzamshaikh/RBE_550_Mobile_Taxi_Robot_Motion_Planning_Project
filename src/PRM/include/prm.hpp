#ifndef PRM_HPP
#define PRM_HPP

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Point.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetPlan.h>
#include <angles/angles.h>

#include <math.h>
#include <random>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include <vector>

using std::string;
using std::vector;

namespace PRM_Planner {
   class Node{
         public:
            Node(int x=0, int y=0, double g=0.0, int index=0, int parent= 0){
                  x_ = x;
                  y_ = y;
                  g_ = g;
                  index_ = index;
                  parent_ = parent;
            }

            int x_, y_, index_, parent_;
            double g_;
   };
   
   class PRM : public nav_core::BaseGlobalPlanner {
      public:
         PRM();
         PRM(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

         void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
         void map2Grid(double mapX, double mapY, int& gridX, int& gridY);
         bool makePlan(const geometry_msgs::PoseStamped& start,
                       const geometry_msgs::PoseStamped& goal,
                       std::vector<geometry_msgs::PoseStamped>& plan) override;
         void map2World(double mapX, double mapY, double& worldX, double& worldY);
         
         int grid2Index(int x, int y);
         void publishExpand(std::vector<Node>& expand);
         void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
         bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
         bool getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan);
         bool runPRM(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);
         void generateRandomSamples(const unsigned char* global_costmap, int num_samples, std::vector<Node>& samples);
         void constructRoadmap(const unsigned char* global_costmap, const std::vector<Node>& samples, int num_neighbors, std::unordered_map<int, std::vector<int>>& roadmap);
         int findNearestNode(const Node& query, const std::vector<Node>& samples);
         bool goalFound(const Node& node, const Node& goal);
         bool searchPath(int start_index, int goal_index, const std::unordered_map<int, std::vector<int>>& roadmap, const std::vector<Node>& samples, std::vector<Node>& path, std::vector<Node>& expand);
         //bool collisionCheck(const Node& n1, const Node& n2,const unsigned char* global_costmap);
         double euclideanDistance(const Node& node1, const Node& node2);
         bool isClearPath(const unsigned char* global_costmap, const Node& node1, const Node& node2);
         bool world2Map(double wx, double wy, double& mx, double& my);
         
      private:
         costmap_2d::Costmap2DROS* costmap_ros_;
         costmap_2d::Costmap2D* costmap_;

         bool initialized_;
         //std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
         //std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
         int num_samples_;
         int max_neighbors_;
         double connection_radius_;
         double originX;
         double originY;
         double resolution;
         double convert_offset;
         string frame_id_;

         Node start_copy;
         Node goal_copy;

         int width;
         int height;
         int mapSize;

         const unsigned char lethal_cost_ = costmap_2d::LETHAL_OBSTACLE;
         const double factor_ = 3.0; // Factor to multiply lethal_cost_ with
         const double max_distance = 15.0; // Maximum distance to connect two nodes
         
         ros::Publisher plan_pub_;  // path planning publisher
         ros::Publisher expand_pub_; // node explorer publisher
         ros::ServiceServer make_plan_srv; // planning service
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

            

      };
}  // namespace PRM_Planner

#endif  // PRM_HPP
