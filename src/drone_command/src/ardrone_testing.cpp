#include<ros/ros.h>
#include "../libs/ardrone.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ardrone_testing_node");
  std::cout << "Getting AR Drone Position" << std::endl;
  ros::NodeHandle nh;

  Ardrone* drone = new Ardrone(nh);
  return 0;
}
