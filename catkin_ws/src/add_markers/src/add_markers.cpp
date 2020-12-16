#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "add_markers/PlaceMarker.h"

// Singleton accessor macro
#define ADD_MARKERS AddMarkers::getInstance()

class AddMarkers
{
public:

  // AddMarkers singleton accessor
  static AddMarkers& getInstance() {
      static AddMarkers single_instance; 
      return single_instance;
  }
  
  // Marker placement callback
  static bool handle_place_request(add_markers::PlaceMarker::Request& req,
                                   add_markers::PlaceMarker::Response& res) 
  {
    ADD_MARKERS.marker.action = req.action; 
    res.msg_feedback = ("Removing RViz object");

    if (req.action == visualization_msgs::Marker::ADD) {
      ADD_MARKERS.marker.pose.position.x = req.pose.position.x;
      ADD_MARKERS.marker.pose.position.y = req.pose.position.y;
      ADD_MARKERS.marker.pose.position.z = req.pose.position.z;
      ADD_MARKERS.marker.pose.orientation.x = req.pose.orientation.x;
      ADD_MARKERS.marker.pose.orientation.y = req.pose.orientation.y;
      ADD_MARKERS.marker.pose.orientation.z = req.pose.orientation.z;
      ADD_MARKERS.marker.pose.orientation.w = req.pose.orientation.w;

      res.msg_feedback = ("Placing RViz object: (" + 
                          std::to_string(req.pose.position.x) + ", " +
                          std::to_string(req.pose.position.y) + ")");
    }

    ADD_MARKERS.marker_publisher.publish(ADD_MARKERS.marker);
    ROS_INFO_STREAM(res.msg_feedback);
    return true;   
  }

  // Cb initialize
  void initialize() 
  {
    marker_publisher = _n.advertise<visualization_msgs::Marker>("/rviz/place_marker", 1);
    placement_service = _n.advertiseService("/add_markers/place_marker", handle_place_request);

    // Initialze marker
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration();
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Marker scale (meters)
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Marker color
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    
    marker.type = visualization_msgs::Marker::CUBE;
    ROS_INFO_STREAM("Add Marker Node initialization completed");
  }

  // Explicitly remove copy ctor and assign op
  AddMarkers(AddMarkers const&)   = delete;
  void operator=(AddMarkers const&) = delete;

private:
    ros::NodeHandle _n;
    ros::Publisher marker_publisher;
    ros::ServiceServer placement_service;
    visualization_msgs::Marker marker;

    // Ctor private
    AddMarkers() {}
}; 

int main(int argc, char **argv) 
{
    // Initiate ROS and AddMarkers instance
    ros::init(argc, argv, "add_markers");
    ADD_MARKERS.initialize();
    ros::spin(); // Eventloop
    return 0;
}

// marker.action = visualization_msgs::Marker::ADD;
// marker.action = visualization_msgs::Marker::DELETE;

// // Marker pose
// marker.pose.position.x = 0;
// marker.pose.position.y = 0;
// marker.pose.position.z = 0;
// marker.pose.orientation.x = 0.0;
// marker.pose.orientation.y = 0.0;
// marker.pose.orientation.z = 0.0;
// marker.pose.orientation.w = 1.0;

