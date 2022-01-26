#include "FlightAutonomy/ImageReceiver.h"

ImageReceiver::ImageReceiver(int _deviceID, ros::NodeHandle &_nh, int _apiID) : deviceID(_deviceID), apiID(_apiID), nh(_nh)
{
}

ImageReceiver::ImageReceiver(ros::NodeHandle &_nh, std::string topic) : nh(_nh)
{
    imgSub = nh.subscribe(topic, 1, &ImageReceiver::imgSubCallback, this);
    camImage = cv::Mat::zeros(cv::Size(640, 640), CV_64FC1);
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


bool ImageReceiver::open()
{
    cap.open(deviceID, apiID);

    if (cap.isOpened())
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void ImageReceiver::receiveImage()
{
    cv::Mat newFrame;
    cap.read(newFrame);

    if (!newFrame.empty())
    {
        camImage = newFrame;
    }
    else
    {
        camImage = cv::Mat::zeros(cv::Size(640, 640), CV_64FC1);
    }
}

void ImageReceiver::setDevice(int _deviceID, int _apiID)
{
    deviceID = _deviceID;
    apiID = _apiID;
}

cv::Mat ImageReceiver::getImage()
{
    return camImage;
}