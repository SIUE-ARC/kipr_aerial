/*
 * Command
 * Main Drone controll program
 *
 * Ryan Owens
 *
 */

 #include <ros/ros.h>

 int main(int argc, char const *argv[]) {

   ros::init(argc, argv, "command");
   std::cout << "Starting the Drone. Stand Clear." << std::endl;
   ros::NodeHandle nh;

  // Other Drone stuff
   return 0;
 }
