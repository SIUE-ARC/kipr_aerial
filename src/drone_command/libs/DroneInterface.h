#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>

class DroneInterface {
public:
  // Transform representing the Drone's Position
  geometry_msgs::TransformStamped positionTransform;
  // Core of ROS
  ros::NodeHandle nh;

  DroneInterface (ros::NodeHandle nh) {
    this->nh = nh;
  }
  // Takeoff comamnd
  virtual void takeoff() = 0;
  // Land command
  virtual void land() = 0;
  // Go To X, Y, Z Position command
  virtual void goTo(int x, int y, int z) = 0;
};
