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
 #include "../libs/ardrone.h"


int main(int argc, char **argv) {
        // Ros Setup
        ros::init(argc, argv, "drone_command");
        std::cout << "Starting the Drone. Stand Clear." << std::endl;
        ros::NodeHandle nh;

        // Publishers
        ros::Publisher drone_velocity_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

        // Drone Setup
        Ardrone* drone = new Ardrone(nh);
        drone->takeoff();

        // Transform Setup
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        geometry_msgs::TransformStamped camera_transform;


        bool done = false;
        bool bottomCamera = true;
        int landed_altitude = 0.5; // Meters
        ros::Rate rate(50);
        while(nh.ok() && !done)
        {
                // Get Transform to Marker from Camera
                try {
                        if(bottomCamera)
                        {
                                camera_transform = tfBuffer.lookupTransform("bottom_camera_marker_0", "world", ros::Time(0));
                        }else
                        {
                                camera_transform = tfBuffer.lookupTransform("front_camera_marker_0", "world", ros::Time(0));
                        }
                }catch(tf2::TransformException &ex)
                {
                        ROS_WARN("%s",ex.what());
                        ros::Duration(1.0).sleep();
                        continue;
                }

                // GET marker transform in Drone Body FOR
                // Get the velocity of the Marker
                geometry_msgs::Twist vel_msg;

                // Attempt to follow...
                drone_velocity_pub.publish(vel_msg);
                ros::spinOnce();

                // Check if on target...
                if(drone->getAltitude() <= landed_altitude)
                {
                        // TODO what to do when landed?
                        done = true; // For now, just say done
                }
        }
        drone->land();
        delete(drone);

        return 0;
}
