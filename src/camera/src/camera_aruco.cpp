/*
 * Camera Aruco
 * Get Aruco Marker positions from the camera
 *
 * Ryan Owens
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "aruco.h"

int main(int argc, char const **argv) {
  // initialize ROS and node handler
  ros::init(argc, argv, "camera_aruco");
  ros::NodeHandle nodeHandle;
  // setup the publisher
  ros::Publisher chatter_pub = nodeHandle.advertise<>("aruco_pose", 1000);
  ros::Rate loop_rate(10);
  // Get the Aruco detector
  Aruco::Aruco arucoDetection = new Aruco::Aruco(1);
  while(ros::ok())
  {
    // publish the results
    auto markerIds = arucoDetection->arucoMarkersInView();
    auto poses = new std::vector<std::vector<double>>();
    for(auto &elem : markerIds) {
      poses.push_back(arucoDetection->getPose(elem));
    }
    // publish the data here
    ros::spinOnce();
    loop_rate.sleep();
  }



  return 0;
}
