/*
 * Command
 * Main Drone controll program
 *
 * Ryan Owens
 *
 */

 #include <ros/ros.h>
 #include <tf2_ros/transform_listener.h>
 #include <geometry_msgs/TransformStamped.h>
 #include <geometry_msgs/Twist.h>

 int main(int argc, char **argv) {

   ros::init(argc, argv, "command");
   std::cout << "Starting the Drone. Stand Clear." << std::endl;
   ros::NodeHandle nh;
   std::string current = "2";

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  while(nh.ok())
  {
    geometry_msgs::TransformStamped transformStamped;
    try {
      transformStamped = tfBuffer.lookupTransform("world", current, ros::Time(0));
    }catch(tf2::TransformException &ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    std::cout << "Transform: " << std::endl;
    std::cout << "Tx " << transformStamped.transform.translation.x << std::endl;
    std::cout << "Ty " << transformStamped.transform.translation.y << std::endl;
    std::cout << "Tz " << transformStamped.transform.translation.z << std::endl;
  }
  // Other Drone stuff
   return 0;
 }
