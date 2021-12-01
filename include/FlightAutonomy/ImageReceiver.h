#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

/**
 * @brief Image receiving class that supports diferrent types of imput video streams. 
 * It converts input video stream into OpenCV mat
 */
class ImageReceiver
{
    ros::NodeHandle &nh; /**< Handle to node */
    ros::Subscriber imageSub; /**< Subscriber to ros camera image topic */
    std::string cameraTopic; /**< Name of camera topic */
    cv::Mat cameraImage; /**< Last frame from camera stream */

public:
    ImageReceiver(ros::NodeHandle &_nh, std::string topic);
    ImageReceiver();
    ~ImageReceiver();

private:

    void imgSubCallback(const sensor_msgs::ImageConstPtr &img);
};