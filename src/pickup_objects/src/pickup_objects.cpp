#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <array>
#include <algorithm>
#include <random>

// Define global pickup and dropoff variables
// float siteA[3] = {0, 3.5, 1.0};
// float siteB[3] = {6.0, 2, 1.0};
// float siteC[3] = {4.5, 4, 1.0};
// float siteD[3] = {-0.75, -5, 1.0};
// float siteE[3] = {-6, 2.5, 1.0};
// bool arrived_at_pickup = true;
// float dropoff[3] = {-2, 5, 1.0};
// bool arrived_at_dropoff = true;

// float sites[5] = {siteA,siteB,siteC,siteD,siteE};

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class TaxiService{

  private:
    float siteA[3] = {0, 3.5, 1.0};
    float siteB[3] = {6.0, 2, 1.0};
    float siteC[3] = {4.5, 4, 1.0};
    float siteD[3] = {-0.75, -5, 1.0};
    float siteE[3] = {-6, 2.5, 1.0};
    //float sites[5] = {siteA,siteB,siteC,siteD,siteE};

    std::vector<std::array<float,3>> testsites;
    std::vector<std::array<float,3>> pickUpSites;




    float dropoff[3] = {-1.8, 6.8, 1};
    bool arrived_at_pickup = true;
    bool arrived_at_dropoff = true;

    

    ros::Publisher goal_pub;

    std_msgs::Bool reached;

    move_base_msgs::MoveBaseGoal goal;

    //actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

    unsigned seed = 0;
    



  public:
    TaxiService(ros::NodeHandle *nh){
      goal_pub = nh->advertise<std_msgs::Bool>("reached_goal", 10);
      testsites.push_back({-0.5, 2.5, 1.0});
      testsites.push_back({6.0, 2, 1.0});

      pickUpSites.push_back({-0.5, 2.5, 1.0});
      pickUpSites.push_back({6.0, 2, 1.0});
      pickUpSites.push_back({4.5, -4, 1.0});
      pickUpSites.push_back({-0.75, -7, 1.0});
      pickUpSites.push_back({-6, 2.5, 1.0});

    }

    bool runService(){
      ROS_INFO("Running Taxi Service!");
      //tell the action client that we want to spin a thread by default
      MoveBaseClient ac("move_base", true);

      // Wait 5 sec for move_base action server to come up
      while(!ac.waitForServer(ros::Duration(5))){
        ROS_INFO("Waiting for the move_base action server to come up");
      }
      // set up the frame parameters
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      // shuffle test sites

      std::shuffle(pickUpSites.begin(), pickUpSites.end(), std::default_random_engine(seed));

      for (auto& site : pickUpSites){
        goal.target_pose.pose.position.x = site[0];
        goal.target_pose.pose.position.y = site[1];
        goal.target_pose.pose.orientation.w = site[2];
    

        // Send the goal position and orientation for the robot to reach
        ROS_INFO("Sending the robot the pickup location!");
        ac.sendGoal(goal);
        ROS_INFO("Robot is on the way!");

        // Wait an infinite time for the results
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
          ROS_INFO("Hooray, the robot is at the pickup location!");
          //ROS_INFO("Publishing pickup message!");

          // Publish data 
          reached.data = true;
          goal_pub.publish(reached);
          /*if(reached.data == true){
            ROS_INFO("---SHOULD HAVE PUBLISHED DATA---");
          }*/
          
          // Sleep for 5 seconds aka pick up the object
          ROS_INFO("Picking up object right now...");
          ros::Duration(5.0).sleep();
          ROS_INFO("Pickup complete!");
          // Change status to false
          reached.data = false;

          // --- DROPOFF LOCATION ---

          // Define a position and orietnation for the robot to reach
          goal.target_pose.pose.position.x = dropoff[0];
          goal.target_pose.pose.position.y = dropoff[1];
          goal.target_pose.pose.orientation.w = dropoff[2];

          // Send the goal position and orientation for the robot to reach
          ROS_INFO("Sending the robot the dropoff location!");
          ac.sendGoal(goal);
          ROS_INFO("Robot is on the way!");

          // Wait an infinite time for the results
          ac.waitForResult();

          // Check if the robot has reached its goal
          if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO("Hooray, the robot is at the dropped off location!");
            //ROS_INFO("Publishing dropoff message!");
            ROS_INFO("Dropping the object right now...");
            // Publish data
            reached.data = true;
            goal_pub.publish(reached);
            ROS_INFO("Dropoff Complete!");

            /*
            if(reached.data == true){
              ROS_INFO("---SHOULD HAVE PUBLISHED DATA---");
            }*/

            ROS_INFO("--- Pickup and dropoff complete! ---");
            ros::Duration(5.0).sleep();

            
          }
          else{
            ROS_INFO("The base failed to move to the drop off location for some reason.");
          }
        }
        else{
          ROS_INFO("The base failed to move to the pick up location for some reason.");
        }
      }

      return true;
      
    }

};



int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pickup_objects");

  // Create a handle to the pick_objects node
  ros::NodeHandle n;

  TaxiService taxi = TaxiService(&n);
  ROS_INFO("Taxi service successfully initialized!");
  while(ros::ok()){
    bool complete = taxi.runService();
    if(complete == true){
      break;
    }

  }  
  ROS_INFO("Taxi service completed successfully!");
  

  //ros::Publisher goal_pub = n.advertise<std_msgs::Bool>("reached_goal", 10);
  //std_msgs::Bool reached;

  //tell the action client that we want to spin a thread by default
  //MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  //while(!ac.waitForServer(ros::Duration(5.0))){
  //  ROS_INFO("Waiting for the move_base action server to come up");
  //}

  //move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  //goal.target_pose.header.frame_id = "map";
  //goal.target_pose.header.stamp = ros::Time::now();

  // --- PICKUP LOCATION ---

  // Define a position and orientation for the robot to reach
  //goal.target_pose.pose.position.x = pickup[0];
  //goal.target_pose.pose.position.y = pickup[1];
  //goal.target_pose.pose.orientation.w = pickup[2];

   // Send the goal position and orientation for the robot to reach
  // ROS_INFO("Sending the robot the pickup location!");
  // ac.sendGoal(goal);
  // ROS_INFO("Robot is on the way!");

  // // Wait an infinite time for the results
  // ac.waitForResult();

  // Check if the robot reached its goal
  // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
  //   ROS_INFO("Hooray, the robot is at the pickup location!");
  //   //ROS_INFO("Publishing pickup message!");

  //   // Publish data 
  //   reached.data = true;
  //   goal_pub.publish(reached);
  //   /*if(reached.data == true){
  //     ROS_INFO("---SHOULD HAVE PUBLISHED DATA---");
  //   }*/
    
  //   // Sleep for 5 seconds aka pick up the object
  //   ROS_INFO("Picking up object right now...");
  //   ros::Duration(5.0).sleep();
  //   ROS_INFO("Pickup complete!");
  //   // Change status to false
  //   reached.data = false;

  //   // --- DROPOFF LOCATION ---

  //   // Define a position and orietnation for the robot to reach
  //   goal.target_pose.pose.position.x = dropoff[0];
  //   goal.target_pose.pose.position.y = dropoff[1];
  //   goal.target_pose.pose.orientation.w = dropoff[2];

  //   // Send the goal position and orientation for the robot to reach
  //   ROS_INFO("Sending the robot the dropoff location!");
  //   ac.sendGoal(goal);
  //   ROS_INFO("Robot is on the way!");

  //   // Wait an infinite time for the results
  //   ac.waitForResult();

  //   // Check if the robot has reached its goal
  //   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
  //     ROS_INFO("Hooray, the robot is at the dropped off location!");
  //     //ROS_INFO("Publishing dropoff message!");
  //     ROS_INFO("Dropping the object right now...");
  //     // Publish data
  //     reached.data = true;
  //     goal_pub.publish(reached);
  //     ROS_INFO("Dropoff Complete!");

  //     /*
  //     if(reached.data == true){
  //       ROS_INFO("---SHOULD HAVE PUBLISHED DATA---");
  //     }*/

  //     ROS_INFO("--- Pickup and dropoff complete! ---");
  //     ros::Duration(5.0).sleep();
  //   }
  //   else{
  //     ROS_INFO("The base failed to move to the drop off location for some reason.");
  //   }
  // }
  // else{
  //   ROS_INFO("The base failed to move to the pick up location for some reason.");
  // }

  
}