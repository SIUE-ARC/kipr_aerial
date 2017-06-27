#include<ros/ros.h>
#include "../libs/ardrone.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "ardrone_testing_node");
  std::cout << "Getting AR Drone Position" << std::endl;
  ros::NodeHandle nh;

  Ardrone* drone = new Ardrone(nh);
  drone->takeoff();
  std::cout << " Takeoff " << std::endl;
  ros::Duration(5.0).sleep();
  // for (size_t i = 0; i < 2; i++) {
    drone->goTo(-20, 0.0, 0.0);
    std::cout << " Go To -20 0 0" << std::endl;
    ros::Duration(1.0).sleep();
  // }
  // drone->hover();
  // std::cout << "Hover" << std::endl;
  // ros::Duration(2.0).sleep();
  drone->land();
  std::cout << "Land" << std::endl;
  ros::Duration(1.0).sleep();
  // ros::Publisher takeoff_pub = nh.advertise<std_msgs::Empty>("/ardrone/takeoff", 10);
  // ros::Publisher landing_pub = nh.advertise<std_msgs::Empty>("/ardrone/land", 10);
  // ros::Publisher move_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  // ros::Duration(2.0).sleep();
  // for (size_t i = 0; i < 15; i++) {
  //   takeoff_pub.publish(std_msgs::Empty());
  // }
  // geometry_msgs::Twist twist;
  // twist.linear.x = 0;
  // twist.linear.y = 0;
  // twist.linear.z = 0;
  // twist.angular.x = 0;
  // twist.angular.y= 0;
  // twist.angular.z = 0;
  // ros::Duration(5.0).sleep();
  // for (size_t i = 0; i < 15; i++) {
  //   move_pub.publish(twist);
  // }
  // std::cout << " Hover " << std::endl;
  // ros::Duration(2.0).sleep();
  //   for (size_t i = 0; i < 15; i++) {
  //   landing_pub.publish(std_msgs::Empty());
  // }
  return 0;
}
