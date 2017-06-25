#include "ardrone.h"

Ardrone::Ardrone(ros::NodeHandle nh)
{
  // Variable init
  this.nh = nh;
  this.max_altitude = 5; // meters
  this.min_altitude = 0; // meters
  this.max_x = 10; // meters
  this.max_y = 4; // meters
  this.max_z = 2; // meters

  this.initPublisherSubscribers();
}

Ardrone::Ardrone (ros::NodeHandle nh, int max_alt, int min_alt, int max_x, int max_y, int max_z)
{
  // Variable init
  this.nh = nh;
  this.max_altitude = max_alt; // meters
  this.min_altitude = min_alt; // meters
  this.max_x = max_x; // meters
  this.max_y = max_y; // meters
  this.max_z = max_z; // meters

  this.initPublisherSubscribers();
}

Ardrone::initPublisherSubscribers()
{
  this.odometry_sub = this.nh.subscribe<std_msgs::Odometry>("ardrone/odometry", 50, this.odometryCallback);
  this.takeoff_pub = this.nh.advertize<std_msgs::Empty>("ardrone/takeoff", 50);
  this.landing_pub = this.nh.advertize<std_msgs::Emtpy>("ardrone/landing", 50);
  this.reset_pub = this.nh.advertize<std_msgs::Emtpy>("ardrone/reset", 50);
  this.move_pub = this.nh.advertize<geometry_msgs::Twist>("ardrone/cmd_vel", 50);

  // this.camera_switcher_service = this.nh.serviceClient<uint8>("ardrone/setcamchannel");
}

Ardrone::odometryCallback(const nav_msgs::Odometry::ConstPtr& pos)
{
  if(DEBUG)
  {
    std::cout << "X: " <<  pos->pose.pose.position.x << " Y: ";
    std::cout <<  pos->pose.pose.position.y << " Z: ";
    std::cout <<  pos->pose.pose.position.z << std::endl;
  }

  if(!DEBUG)
  {
    // TODO do stuff
  }
}

Ardrone::takeoff()
{

}

Ardrone::land()
{

}

Ardrone::reset()
{

}

Ardrone::goTo(int x, int y, int z)
{

}

Ardrone::hover()
{

}
