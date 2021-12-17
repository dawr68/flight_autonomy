#include "FlightAutonomy/ObjectDetector.h"

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
}