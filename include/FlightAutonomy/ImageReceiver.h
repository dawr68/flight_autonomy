#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

/**
 * @brief Image receiving class that supports diferrent types of imput video streams.
 * It converts input video stream into OpenCV mat
 */
class ImageReceiver
{
    ros::NodeHandle &nh;    /**< Handle to node */
    ros::Subscriber imgSub; /**< Subscriber to ros camera image topic */
    std::string camTopic;   /**< Name of camera topic */
    cv::Mat camImage;       /**< Last frame from camera stream */

public:
    ImageReceiver(ros::NodeHandle &_nh, std::string topic);
    ~ImageReceiver();

    cv::Mat getImage();

private:
    void imgSubCallback(const sensor_msgs::ImageConstPtr &img);
};