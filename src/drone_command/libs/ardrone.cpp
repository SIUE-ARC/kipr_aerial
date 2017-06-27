#include "ardrone.h"

Ardrone::Ardrone(ros::NodeHandle nh) : DroneInterface(nh)
{
        // Variable init
        this->max_altitude = 5; // meters
        this->min_altitude = 0; // meters
        this->max_x = 10; // meters
        this->max_y = 4; // meters
        this->max_z = 2; // meters
        this->state = States::UNKNOWN;

        this->initPublisherSubscribers();
}

Ardrone::Ardrone (ros::NodeHandle nh, int max_alt, int min_alt, int max_x, int max_y, int max_z) : DroneInterface(nh)
{
        // Variable init
        this->max_altitude = max_alt; // meters
        this->min_altitude = min_alt; // meters
        this->max_x = max_x; // meters
        this->max_y = max_y; // metersg
        this->max_z = max_z; // meters
        this->state = States::UNKNOWN;

        this->initPublisherSubscribers();
}

Ardrone::~Ardrone()
{
        if(this->state != States::LANDED)
        {
                this->land();
        }
}

void Ardrone::initPublisherSubscribers()
{
        this->odometry_sub = this->nh.subscribe<nav_msgs::Odometry>("ardrone/odometry", 50, this->odometryCallback);
        this->legacynav_sub = this->nh.subscribe<ardrone_autonomy::Navdata>("ardrone/navdata", 50, &Ardrone::navdataCallback, this);
        this->takeoff_pub = this->nh.advertise<std_msgs::Empty>("ardrone/takeoff", 50);
        this->landing_pub = this->nh.advertise<std_msgs::Empty>("ardrone/landing", 50);
        this->reset_pub = this->nh.advertise<std_msgs::Empty>("ardrone/reset", 50);
        this->move_pub = this->nh.advertise<geometry_msgs::Twist>("ardrone/cmd_vel", 50);

        // this->camera_switcher_service = this->nh.serviceClient<uint8>("ardrone/setcamchannel");
}

void Ardrone::odometryCallback(const nav_msgs::Odometry::ConstPtr& pos)
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

void Ardrone::navdataCallback(const ardrone_autonomy::Navdata::ConstPtr& data)
{
        if(DEBUG)
        {
                this->printNavData(*data);
        }

        this->navdata = *data;
        this->state = (States)data->state;
}

void Ardrone::printNavData(ardrone_autonomy::Navdata data)
{
        std::cout << "Nav Data: " << std::endl;
        std::cout << "Battery Percentage: " << data.batteryPercent << "%"<< std::endl;
        std::cout << "State: " << data.state << std::endl;
        std::cout << "Rotation in the X (left/right): " << data.rotX << " degrees" << std::endl;
        std::cout << "Rotation in the Y (forward/backwards): " << data.rotY << " degrees" << std::endl;
        std::cout << "Rotation in the Z (orientation): " << data.rotZ << " degrees" << std::endl;
        std::cout << "Magnetometer: X" << data.magX << " Y " << data.magY << " Z " << data.magZ << std::endl;
        std::cout << "Air Pressure: " << data.pressure << " Pa" << std::endl;
        std::cout << "Air Temperature: " << data.temp << " UNIT" << std::endl;
        std::cout << "Wind Speed: " << data.wind_speed << " UNIT" << std::endl;
        std::cout << "Wind Angle: " << data.wind_angle << " UNIT" << std::endl;
        std::cout << "Wind Componsation Angle: " << data.wind_comp_angle << " UNIT" << std::endl;
        std::cout << "Altitude: " << (0.001 * data.altd) << " M" << std::endl;
        std::cout << "Motor 1 PWM: " << data.motor1 << std::endl;
        std::cout << "Motor 2 PWM: " << data.motor2 << std::endl;
        std::cout << "Motor 3 PWM: " << data.motor3 << std::endl;
        std::cout << "Motor 4 PWM: " << data.motor4 << std::endl;
        std::cout << "Linear Velocity X: " << (0.001 * data.vx) << " m/s" << std::endl;
        std::cout << "Linear Velocity Y: " << (0.001 * data.vy) << " m/s" << std::endl;
        std::cout << "Linear Velocity Z: " << (0.001 * data.vz) << " m/s" << std::endl;
        std::cout << "Linear Acceleration X: " << (0.001 * data.ax) << " g" << std::endl;
        std::cout << "Linear Acceleration Y: " << (0.001 * data.ay) << " g" << std::endl;
        std::cout << "Linear Acceleration Z: " << (0.001 * data.az) << " g" << std::endl;
        std::cout << std::endl;
}

void Ardrone::takeoff()
{
        if(!DEBUG)
        {
                if(this->state != States::FLYING_1 && this->state != States::FLYING_2 && this->state != States::LANDING)
                {
                        // std_msgs::Empty msg;
                        for (size_t i = 0; i < 5; i++) {
                                this->takeoff_pub.publish(std_msgs::Empty());
                        }
                        ros::spinOnce();
                }
        }
}

void Ardrone::land()
{
        if(!DEBUG)
        {
                if(this->state == States::FLYING_1 || this->state == States::FLYING_2)
                {
                        // std_msgs::Empty msg;
                        for (size_t i = 0; i < 5; i++) {
                                this->landing_pub.publish(std_msgs::Empty());
                        }
                        ros::spinOnce();
                }
        }
}

void Ardrone::reset()
{
        if(!DEBUG)
        {
                if(this->state == States::FLYING_1 || this->state == States::FLYING_2 || this->state == States::TAKING_OFF)
                {
                        ROS_DEBUG("RESET WHEN CURRENTLY ACTIVE, CAUTION");
                }
                for (size_t i = 0; i < 5; i++) {
                        this->reset_pub.publish(std_msgs::Empty());
                }
                ros::spinOnce();
        }
}

void Ardrone::goTo(int x, int y, int z)
{
        if(!DEBUG)
        {
                if((this->state == States::FLYING_1 || this->state == States::FLYING_2 ) && x < this->max_x && y < this->max_y && z < this->max_z)
                {
                        geometry_msgs::Twist twist;
                        twist.linear.x = x;
                        twist.linear.y = y;
                        twist.linear.z = z;
                        this->move_pub.publish(twist);
                }else
                {
                        std::cerr << "Ignore GoTo Command" << std::endl;
                }

        }

}

void Ardrone::hover()
{
        if(!DEBUG)
        {
                if(this->state == States::FLYING_1 || this->state == States::FLYING_2)
                {
                        geometry_msgs::Twist twist;
                        twist.linear.x = 0;
                        twist.linear.y = 0;
                        twist.linear.z = 0;
                        twist.angular.z = 0;
                        this->move_pub.publish(twist);
                }
        }
}
