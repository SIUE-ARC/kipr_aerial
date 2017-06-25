#include <ros/ros.h>
#include <std_msgs/Emtpy.h>
#include <nav_msgs/Odometry.h>
#include <geomevry_msgs/Twist.h>
#include <string>


#define DEBUG 1
class Ardrone {
private:
  // Core of ROS
  ros::NodeHandle nh;
  // Subscribers
  ros::Subscriber odometry_sub;
  // ros::Subscriber front_camera_sub; // Not needed if the camera node is running
  // ros::Subscriber bottom_camera_sub; // Not needed if the camera node is running

  // Publishers
  ros::Publisher takoff_pub;
  ros::Publisher landing_pub;
  ros::Publisher reset_pub;
  ros::Publisher move_pub;

  // Service Clients
  ros::ServiceClient camera_switcher_service;

  std::string front_camera_config_file;
  std::string bottom_camera_config_file;

  // Ranges
  int max_altitude, min_altitude;
  int max_x, max_y, max_z;

  // private methods
  void initPublisherSubscribers();

  // callbacks
  void odometryCallback(const nav_msgs::Odometry::ConstPtr& pos);

public:
  Ardrone (ros::NodeHandle nh);
  Ardrone (ros::NodeHandle nh, int max_alt, int min_alt, int max_x, int max_y, int max_z);
  virtual ~Ardrone ();

  // Change State
  void takeoff();
  void land();
  void reset();

  // Movement
  void goTo(int x, int y, int z);
  void hover();
};
