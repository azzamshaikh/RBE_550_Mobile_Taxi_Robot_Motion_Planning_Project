#ifndef PRM_HPP
#define PRM_HPP

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <cmath>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <geometry_msgs/Point.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <visualization_msgs/Marker.h>
#include <set>
#include <nav_msgs/GetPlan.h>
#include <angles/angles.h>
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

    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan) override;
    void map2World(double mapX, double mapY, double& worldX, double& worldY);
    void map2Grid(double mapX, double mapY, int& gridX, int& gridY);
    int grid2Index(int x, int y);
    void index2Grid(int i, int& x, int& y);
    void publishExpand(std::vector<Node>& expand);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& plan);
    bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);
    bool getPlanFromPath(std::vector<Node>& path, std::vector<geometry_msgs::PoseStamped>& plan);
    bool runPRM(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path, std::vector<Node>& expand);
   void generateRandomSamples(const unsigned char* global_costmap, int num_samples, std::vector<Node>& samples);
   void constructRoadmap(const unsigned char* global_costmap, const std::vector<Node>& samples, int num_neighbors, std::unordered_map<int, std::vector<int>>& roadmap);
      int findNearestNode(const Node& query, const std::vector<Node>& samples);
   bool searchPath(int start_index, int goal_index, const std::unordered_map<int, std::vector<int>>& roadmap, const std::vector<Node>& samples, std::vector<Node>& path, std::vector<Node>& expand);
        double euclideanDistance(const Node& node1, const Node& node2);
        bool isClearPath(const unsigned char* global_costmap, const Node& node1, const Node& node2);
        bool indexExistsInDict(std::unordered_map<int, Node>& closedList, Node &n);
      bool isIllegalNode(const unsigned char* global_costmap, Node &n, Node &current);
      
    
private:
    costmap_2d::Costmap2DROS* costmap_ros_;
    costmap_2d::Costmap2D* costmap_;
    bool initialized_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
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
        const double max_distance = 10.0; // Maximum distance to connect two nodes
    std::vector<geometry_msgs::Point> generateRoadmap(const geometry_msgs::PoseStamped& start,
                                                      const geometry_msgs::PoseStamped& goal);

    std::vector<geometry_msgs::PoseStamped> findPath(const geometry_msgs::Point& start,
                                                     const geometry_msgs::Point& goal,
                                                     const std::vector<geometry_msgs::Point>& roadmap);

    double randomDouble(double min, double max);
    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2);
    bool isValidPoint(const geometry_msgs::Point& point);
    bool worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my);
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
