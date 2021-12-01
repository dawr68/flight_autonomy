#include "FlightAutonomy/ImageReceiver.h"

ImageReceiver::ImageReceiver(ros::NodeHandle &_nh, std::string topic) : nh(_nh), cameraTopic(topic)
{
    _nh.subscribe(cameraTopic, 1, &ImageReceiver::imgSubCallback, this);
}

ImageReceiver::~ImageReceiver()
{
}

void ImageReceiver::imgSubCallback(const sensor_msgs::ImageConstPtr &img)
{
    cv_bridge::CvImagePtr cvPtr;

    try
    {
        cvPtr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}