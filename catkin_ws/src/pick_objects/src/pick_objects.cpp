#include <thread>
#include <atomic>
#include <mutex>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "add_markers/PlaceMarker.h"

#define PICK_OBJECTS PickObjects::getInstance() // Singleton accessor macro
#define XY_TOLERANCE 0.5                        // Positional zone tolerance

// SimpleActionClient typdef
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

enum DeliveryMode { 
    APPROACH, 
    WAIT
};


class PickObjects
{
public:

  // PickObjects singleton accessor
  static PickObjects& getInstance() {
      static PickObjects single_instance; 
      return single_instance;
  }
  // Odometry callback (maintain current 2D position)
  static void handle_odometry(nav_msgs::Odometry odom) {
    std::lock_guard<std::mutex> guard(PICK_OBJECTS.positionlock);
    PICK_OBJECTS.position.x = odom.pose.pose.position.x;
    PICK_OBJECTS.position.y = odom.pose.pose.position.y;
  }

  // Marker placement callback
  static void handle_marker_placement(visualization_msgs::Marker marker) 
  {
    geometry_msgs::Point currpos = PICK_OBJECTS.safe_position();
    if (marker.action == visualization_msgs::Marker::ADD &&
        fabs(marker.pose.position.x - currpos.x) > XY_TOLERANCE  &&
        fabs(marker.pose.position.y - currpos.y) > XY_TOLERANCE)
    {
      std::lock_guard<std::mutex> guard(PICK_OBJECTS.targetlock);
      PICK_OBJECTS.target.pose.position.x = marker.pose.position.x;
      PICK_OBJECTS.target.pose.position.y = marker.pose.position.y;
      PICK_OBJECTS.target.pose.orientation.w = marker.pose.orientation.w;
      PICK_OBJECTS.mode = DeliveryMode::APPROACH;
    }
  }

  // PickObjects state machine event loop. Executes on its own thread and 
  // communicates with ros::spin thread ONLY through specified shared resources.
  void state_machine(ros::Rate loop_rate)
  {
    while (ros::ok()) {
      switch(mode) 
      {
        case DeliveryMode::APPROACH:
        {
          move_base_msgs::MoveBaseGoal goal; populate_goal(goal);
          ROS_INFO("Virtual object ready for pickup");
          ROS_INFO_STREAM("Proceeding to pickup zone: (" + 
            std::to_string(goal.target_pose.pose.position.x) + ", " +
            std::to_string(goal.target_pose.pose.position.y) + ")");

          // Go to pickup zone
          action_client.sendGoal(goal);
          action_client.waitForResult();

          if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("SUCCESS: arrived at pickup zone");
            ROS_INFO("Picking up virtual object, please wait");

            // Pickup/remove rviz marker
            add_markers::PlaceMarker delmarker;
            delmarker.request.action = visualization_msgs::Marker::DELETE;
            if (marker_client.call(delmarker)) {
              
              ros::Duration(5).sleep(); // Pause at pickup
              goal.target_pose.header.stamp = ros::Time::now();
              goal.target_pose.pose.position.x = dropoff.x;
              goal.target_pose.pose.position.y = dropoff.y;
              ROS_INFO("Pickup completed");
              ROS_INFO_STREAM("Proceeding to dropoff zone: (" + 
                std::to_string(goal.target_pose.pose.position.x) + ", " +
                std::to_string(goal.target_pose.pose.position.y) + ")");

              // Go to dropoff zone
              action_client.sendGoal(goal);
              action_client.waitForResult();
              
              if(action_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                ROS_INFO("SUCCESS! Arrived to dropoff zone. Delivering virtual object.");
                
                // Dropoff/insert rviz marker
                add_markers::PlaceMarker addmarker;
                addmarker.request.action = visualization_msgs::Marker::ADD;
                addmarker.request.pose.position.x = dropoff.x;
                addmarker.request.pose.position.y = dropoff.y;
                addmarker.request.pose.orientation.w = 1.0;

                // Return to waiting zone
                if (marker_client.call(addmarker)) {
                  goal.target_pose.header.stamp = ros::Time::now();
                  goal.target_pose.pose.position.x = waiting.x;
                  goal.target_pose.pose.position.y = waiting.y;
                  action_client.sendGoal(goal);
                  ROS_INFO_STREAM("Proceeding to waiting zone for next delivery request: (" + 
                    std::to_string(goal.target_pose.pose.position.x) + ", " +
                    std::to_string(goal.target_pose.pose.position.y) + ")");
                }
              }
              else ROS_INFO("FAILED to arrive at droppff zone");
            }
            else ROS_ERROR("FAILED to delete marker");
          }
          else ROS_INFO("FAILED to arrive at pickup zone");
          
          mode = DeliveryMode::WAIT;
          break;
        }
        case DeliveryMode::WAIT: break;
        default: ROS_ERROR("Unrecogized delivery mode");
      }

      loop_rate.sleep();
    }
  }
  
  geometry_msgs::Point safe_position() {
      std::lock_guard<std::mutex> guard(positionlock);
      return position;
  }
  
  void populate_goal(move_base_msgs::MoveBaseGoal &goal) {
      std::lock_guard<std::mutex> guard(targetlock);
      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = target.pose.position.x;
      goal.target_pose.pose.position.y = target.pose.position.y;
      goal.target_pose.pose.orientation.w = target.pose.orientation.w;
  }

  // Cb initialize
  void initialize() 
  {
    marker_subscriber = _n.subscribe("/rviz/place_marker", 1, handle_marker_placement);
    marker_client = _n.serviceClient<add_markers::PlaceMarker>("/add_markers/place_marker");
    odom_subscriber = _n.subscribe("/odom", 1, handle_odometry);

    // Wait for move_base action server to come up
    while(!action_client.waitForServer(ros::Duration(2.0))) 
      ROS_INFO("Waiting for the move_base action server to come up");
    ROS_INFO_STREAM("Pick Objects Node initialization completed");
  }

  // Explicitly remove copy ctor and assign op
  PickObjects(PickObjects const&)    = delete;
  void operator=(PickObjects const&) = delete;

private:

    ros::NodeHandle _n;
    ros::Subscriber odom_subscriber;
    ros::Subscriber marker_subscriber;
    ros::ServiceClient marker_client;
    MoveBaseClient action_client;

    // Const drop and waiting zones
    geometry_msgs::Point dropoff;
    geometry_msgs::Point waiting;

    // -------- !!! (Start) THREAD SHARED RESOURCES !!! ---------
    std::atomic<DeliveryMode> mode;
    geometry_msgs::Point position;
    std::mutex positionlock;

    visualization_msgs::Marker target;
    std::mutex targetlock;
    // ---------- !!! (End) THREAD SHARED RESOURCES !!! -----------

    // Ctor private
    PickObjects() 
      : action_client("move_base", true), 
        mode(DeliveryMode::WAIT)
    {
      dropoff.x = -1.0; dropoff.y = 6.0;
      waiting.x = -7.0; waiting.y = 2.0;
    }
}; 

int main(int argc, char **argv) 
{
    // Initiate ROS and PickObjects instance
    ros::init(argc, argv, "pick_objects");
    PICK_OBJECTS.initialize();
    
    // Start robot state machine
    ros::Rate smachine_rate(10); // 10Hz
    std::thread smachine_thrd(
        &PickObjects::state_machine, 
        &PICK_OBJECTS, smachine_rate);

    // Eventloop
    ros::spin();
    smachine_thrd.join();
    return 0;
}
