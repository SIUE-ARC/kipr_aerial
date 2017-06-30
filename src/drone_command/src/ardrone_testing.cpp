#include <ros/ros.h>
#include "../libs/ardrone.h"

int main(int argc, char **argv) {
        ros::init(argc, argv, "ardrone_testing_node");
        std::cout << "Getting AR Drone Position" << std::endl;
        ros::NodeHandle nh;

        Ardrone* drone = new Ardrone(nh);
        drone->takeoff();
        std::cout << " Takeoff " << std::endl;
        ros::Duration(5.0).sleep();

        std::cout << "Hover" << std::endl;
        ros::Duration(5.0).sleep();
        drone->goTo(7.0, 0.0, 0.0);
        drone->goTo(-7.0, 0.0, 0.0);
        drone->land();
        std::cout << "Land" << std::endl;
        ros::Duration(1.0).sleep();
        return 0;
}
