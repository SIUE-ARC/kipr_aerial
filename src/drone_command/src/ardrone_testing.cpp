#include<ros/ros.h>
#include<nav_msgs/Odometry.h>
#include<std_msgs/Empty.h>

void printPosition(const nav_msgs::Odometry::ConstPtr& pos)
{
  std::cout << "X: " <<  pos->pose.pose.position.x << " Y: ";
  std::cout <<  pos->pose.pose.position.y << " Z: ";
  std::cout <<  pos->pose.pose.position.z << std::endl;
}

void takeoff(ros::Publisher pub)
{
  std_msgs::Empty msg;
  for (size_t i = 0; i < 5; i++) {
    pub.publish(msg);
  }
  ros::spin();
}




int main(int argc, char **argv) {
  ros::init(argc, argv, "ardrone_testing_node");
  std::cout << "Getting AR Drone Position" << std::endl;
  ros::NodeHandle nh;

  ros::Subscriber state_sub = nh.subscribe<nav_msgs::Odometry> ("ardrone/odometry", 10, printPosition);

  ros::spin();
  return 0;
}
