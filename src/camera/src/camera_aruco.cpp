/*
 * Camera Aruco
 * Get Aruco Marker positions from the camera
 *
 * Ryan Owens
 *
 */

#include <geometry_msgs/TransformStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "aruco.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "camera");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  std::vector<geometry_msgs::TransformStamped> transforms;
  Aruco *arucoDetector = new Aruco(1);
  if (!arucoDetector->openCamera()) {
    std::cerr << "Could not open the camera" << std::endl;
    return -1;
  }
  while (ros::ok()) {
    std::vector<int> markers = arucoDetector->arucoMarkersInView();
    for (auto &marker : markers) {
      std::vector<double> directPose = arucoDetector->getPose(marker);
      transformStamped.header.stamp = ros::Time::now();
      transformStamped.header.frame_id = "world";
      transformStamped.child_frame_id = std::to_string(marker);
      transformStamped.transform.translation.x = directPose[0];
      transformStamped.transform.translation.y = directPose[1];
      transformStamped.transform.translation.z = directPose[2];
      transformStamped.transform.rotation.x = directPose[3];
      transformStamped.transform.rotation.y = directPose[4];
      transformStamped.transform.rotation.z = directPose[5];
      transformStamped.transform.rotation.w = 1.0;

      transforms.push_back(transformStamped);
    }
    br.sendTransform(transforms);
    transforms.clear();
    ros::spinOnce();
    loop_rate.sleep();
  }
  delete (arucoDetector);

  return 0;
}
