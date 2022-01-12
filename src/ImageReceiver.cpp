#include "FlightAutonomy/ImageReceiver.h"

ImageReceiver(int _deviceID, int _apiID = 0) : deviceID(_deviceID), apiID(_apiID)
{
}

bool ImageReceiver::open()
{
    cap.open(deviceID, apiID);

    if(cap.isOpen())
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
    cap.read(newFrame):

    if(!newFrame.empty())
    {
        camImage = newFrame;
    }
    else
    {
        camImage = cv::Mat::zeros(cv::Size(640, 640), CV_64FC1);
    }
}


cv::Mat ImageReceiver::getImage()
{
    return camImage
}