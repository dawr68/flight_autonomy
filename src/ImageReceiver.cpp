#include "FlightAutonomy/ImageReceiver.h"

ImageReceiver::ImageReceiver(int _deviceID, int _apiID) : deviceID(_deviceID), apiID(_apiID)
{
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