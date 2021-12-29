#include "FlightAutonomy/ObjectDetector.h"
#include <iostream>

ObjectDetector::ObjectDetector() : params(cv::aruco::DetectorParameters::create()),
                                   dict(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000))
{
}

ObjectDetector::ObjectDetector(cv::Ptr<cv::aruco::DetectorParameters> _params, cv::Ptr<cv::aruco::Dictionary> _dict) : params(_params), dict(_dict)
{
}

bool ObjectDetector::detect(const cv::Mat &img)
{
    cv::aruco::detectMarkers(img, dict, markCor, markIds, params, rejected);
    cv::aruco::drawDetectedMarkers(img, markCor, markIds);

    cv::circle(img, calcArucoCenter(markCor[0]), 5, cv::Scalar(0,0,255), cv::FILLED, cv::LINE_8);
}

cv::Vec3f ObjectDetector::getPosition()
{

}

cv::Point2i calcArucoCenter(std::vector<cv::Point2f> corners)
{
    return cv::Point((corners[0] + corners[2]) / 2);
}