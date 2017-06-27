#include <ros/console.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <ardrone_autonomy/Navdata.h>
#include <string>

#include "DroneInterface.h"

#define DEBUG 1
class Ardrone : public DroneInterface {
private:
// Subscribers
ros::Subscriber odometry_sub;
ros::Subscriber legacynav_sub;
// ros::Subscriber front_camera_sub; // Not needed if the camera node is running
// ros::Subscriber bottom_camera_sub; // Not needed if the camera node is running

// Publishers
ros::Publisher takeoff_pub;
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

// States
enum class States {
UNKNOWN = 0,
INITED = 1,
LANDED = 3,
FLYING_1 = 3,
FLYING_2 = 7,
TEST = 5,
TAKING_OFF = 6,
LANDING = 8,
LOOPING = 9
};

// Curent state
States state;

// Current Nav Data
ardrone_autonomy::Navdata navdata;

// private methods
void initPublisherSubscribers();
void printNavData(ardrone_autonomy::Navdata data);

// callbacks
void static odometryCallback(const nav_msgs::Odometry::ConstPtr& pos);
void navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& data);

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

// Status
double getAltitude();
};
