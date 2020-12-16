#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Pickup location position and orientation 
  goal.target_pose.pose.position.x = 2.0;
  goal.target_pose.pose.position.y = -7.0;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending pickup move_base goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("SUCCESS: pickup completed");
  else { ROS_INFO("FAILURE: pickup aborted"); return 0; }

  // Dropoff location position and orientation 
  goal.target_pose.pose.position.x = -1.0;
  goal.target_pose.pose.position.y = 6.0;
  goal.target_pose.pose.orientation.w = 1.0;
  ROS_INFO("Sending dropff move_base goal");
  ac.sendGoal(goal);
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("SUCCESS: dropoff completed");
  else ROS_INFO("FAILURE: dropoff aborted");
  return 0;
}