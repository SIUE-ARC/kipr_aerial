/*
 * Camera Aruco
 * Get Aruco Marker positions from the camera
 *
 * Ryan Owens
 *
 */

#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>

#include "aruco.h"

int main(int argc, char const *argv[]) {
  Aruco *arucoDetector = new Aruco(1);
  delete (arucoDetector);

  return 0;
}
