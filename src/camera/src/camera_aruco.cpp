/*
 * Camera Aruco
 * Get Aruco Marker positions from the camera
 *
 * Ryan Owens
 *
 */

#include <geometry_msgs/TransformStamped.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <ros/ros.h>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "aruco.h"

void publishUsbCamera();
void publishFrontARCamera(const sensor_msgs::ImageConstPtr& msg);
void publishBottomARCamera(const sensor_msgs::ImageConstPtr& msg);
tf2::Quaternion *convertToQuaternion(double rotx, double roty, double rotz);

const static std::string ARDRONE_FRONT_CAMERA_FILE = "ardrone_front.yml";
const static std::string ARDRONE_BOTTOM_CAMERA_FILE = "ardrone_bottom.yml";

int main(int argc, char **argv) {
        ros::init(argc, argv, "camera");
        // Get the Node Handle
        ros::NodeHandle nh;
        // Subscribe to both the front camera and bottom camera
        std::cout << "Using AR Drone cameras" << std::endl;
        image_transport::ImageTransport it(nh);
        image_transport::Subscriber front_sub = it.subscribe("/ardrone/front/image_raw", 0, publishFrontARCamera);
        image_transport::Subscriber bottom_sub = it.subscribe("/ardrone/bottom/image_raw", 0, publishBottomARCamera);
        ros::spin();
        ros::Rate loop_rate(10);
}

tf2::Quaternion *convertToQuaternion(double rotx, double roty, double rotz) {
        float angle =
                (float)(sqrt(rotx * rotx + roty * roty + rotz + rotz) * (180 / M_PI));
        tf2::Vector3 axis(-rotx, roty, -rotz);
        return new tf2::Quaternion(axis, angle);
}

void publishUsbCamera()
{
        ros::Rate loop_rate(10);
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        std::vector<geometry_msgs::TransformStamped> transforms;
        Aruco *arucoDetector = new Aruco(0);
        if (!arucoDetector->openCamera()) {
                std::cerr << "Could not open the camera" << std::endl;
                exit(-1);
        }
        while (ros::ok()) {
                std::vector<int> markers = arucoDetector->arucoMarkersInView();
                for (auto &marker : markers) {
                        std::vector<double> directPose = arucoDetector->getPose(marker);
                        std::cout << std::endl;
                        if (directPose.size() >= 6) {
                                transformStamped.header.stamp = ros::Time::now();
                                transformStamped.header.frame_id = "front_camera";
                                transformStamped.child_frame_id = "front_camera_marker_" + std::to_string(marker);
                                transformStamped.transform.translation.x = directPose[0];
                                transformStamped.transform.translation.y = directPose[1];
                                transformStamped.transform.translation.z = directPose[2];
                                tf2::Quaternion *q =
                                        convertToQuaternion(directPose[3], directPose[4], directPose[5]);
                                transformStamped.transform.rotation.x = q->x();
                                transformStamped.transform.rotation.y = q->y();
                                transformStamped.transform.rotation.z = q->z();
                                transformStamped.transform.rotation.w = q->w();
                                transforms.push_back(transformStamped);
                                delete (q);
                                q = nullptr;
                        }
                }
                br.sendTransform(transforms);
                transforms.clear();
                ros::spinOnce();
                loop_rate.sleep();
        }
        delete (arucoDetector);
        arucoDetector = nullptr;
}

void publishFrontARCamera(const sensor_msgs::ImageConstPtr& msg)
{
        ros::Rate loop_rate(10);
        cv::Mat frontCameraFrame = cv_bridge::toCvShare(msg)->image;
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        std::vector<geometry_msgs::TransformStamped> transforms;
        Aruco *arucoDetector = new Aruco(0, ARDRONE_FRONT_CAMERA_FILE);

        std::vector<int> markers = arucoDetector->arucoMarkersInView(&frontCameraFrame);
        for (auto &marker : markers) {
                std::vector<double> directPose = arucoDetector->getPose(marker, &frontCameraFrame);
                std::cout << std::endl;
                if (directPose.size() >= 6) {
                        std::cout << "Publising Marker " << std::to_string(marker) << std::endl;
                        transformStamped.header.stamp = ros::Time::now();
                        transformStamped.header.frame_id = "front_camera";
                        transformStamped.child_frame_id = "front_camera_marker_" + std::to_string(marker);
                        transformStamped.transform.translation.x = directPose[0];
                        transformStamped.transform.translation.y = directPose[1];
                        transformStamped.transform.translation.z = directPose[2];
                        tf2::Quaternion *q =
                                convertToQuaternion(directPose[3], directPose[4], directPose[5]);
                        transformStamped.transform.rotation.x = q->x();
                        transformStamped.transform.rotation.y = q->y();
                        transformStamped.transform.rotation.z = q->z();
                        transformStamped.transform.rotation.w = q->w();
                        transforms.push_back(transformStamped);
                        delete (q);
                        q = nullptr;
                }
        }
        br.sendTransform(transforms);
        transforms.clear();
        ros::spinOnce();
        loop_rate.sleep();
        delete arucoDetector;
        arucoDetector = nullptr;

}

void publishBottomARCamera(const sensor_msgs::ImageConstPtr& msg)
{
        ros::Rate loop_rate(10);
        cv::Mat bottomCameraFrame = cv_bridge::toCvShare(msg)->image;
        static tf2_ros::TransformBroadcaster br;
        geometry_msgs::TransformStamped transformStamped;
        std::vector<geometry_msgs::TransformStamped> transforms;
        Aruco *arucoDetector = new Aruco(0, ARDRONE_BOTTOM_CAMERA_FILE);

        std::vector<int> markers = arucoDetector->arucoMarkersInView(&bottomCameraFrame);
        for (auto &marker : markers) {
                std::vector<double> directPose = arucoDetector->getPose(marker, &bottomCameraFrame);
                std::cout << std::endl;
                if (directPose.size() >= 6) {
                        std::cout << "Publising Marker " << std::to_string(marker) << std::endl;
                        transformStamped.header.stamp = ros::Time::now();
                        transformStamped.header.frame_id = "bottom_camera";
                        transformStamped.child_frame_id = "bottom_camera_marker_" + std::to_string(marker);
                        transformStamped.transform.translation.x = directPose[0];
                        transformStamped.transform.translation.y = directPose[1];
                        transformStamped.transform.translation.z = directPose[2];
                        tf2::Quaternion *q =
                                convertToQuaternion(directPose[3], directPose[4], directPose[5]);
                        transformStamped.transform.rotation.x = q->x();
                        transformStamped.transform.rotation.y = q->y();
                        transformStamped.transform.rotation.z = q->z();
                        transformStamped.transform.rotation.w = q->w();
                        transforms.push_back(transformStamped);
                        delete (q);
                        q = nullptr;
                }
        }
        br.sendTransform(transforms);
        transforms.clear();
        ros::spinOnce();
        loop_rate.sleep();

        delete arucoDetector;
        arucoDetector = nullptr;
}
