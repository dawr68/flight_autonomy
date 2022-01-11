#include "FlightAutonomy/ImageReceiver.h"

ImageReceiver::ImageReceiver(ros::NodeHandle &_nh, std::string topic) : nh(_nh), camTopic(topic)
{
    imgSub = nh.subscribe(camTopic, 1, &ImageReceiver::imgSubCallback, this);
    camImage = cv::Mat::zeros(cv::Size(640, 640), CV_64FC1);
}

ImageReceiver::~ImageReceiver()
{
}

void ImageReceiver::imgSubCallback(const sensor_msgs::ImageConstPtr &img)
{
    try
    {
        cv_bridge::CvImagePtr cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
        camImage = cvPtr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

cv::Mat ImageReceiver::getImage()
{
    return camImage;
}