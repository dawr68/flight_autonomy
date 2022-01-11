#include "FlightAutonomy/ObjectDetector.h"
#include <iostream>

ObjectDetector::ObjectDetector() : params(cv::aruco::DetectorParameters::create()),
                                   dict(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_1000))
{
}

ObjectDetector::ObjectDetector(cv::Ptr<cv::aruco::DetectorParameters> _params, cv::Ptr<cv::aruco::Dictionary> _dict)
    : params(_params), dict(_dict)
{
}

bool ObjectDetector::detect(const cv::Mat &img)
{
    cv::aruco::detectMarkers(img, dict, markCor, markIds, params, rejected);

#ifdef FA_DEBUG
    cv::aruco::drawDetectedMarkers(img, markCor, markIds);
#endif

    if (markIds.size() == 0)
    {
        objPostion = cv::Point2i(-1, -1);
        return 0;
    }

    for (auto mCor : markCor)
    {
        objPostion = calcArucoCenter(mCor);
    }

    return 1;
}

cv::Vec3f ObjectDetector::getPosition()
{
    return cv::Vec3f(objPostion.x, objPostion.y, 0.f);
}

cv::Point2i calcArucoCenter(std::vector<cv::Point2f> corners)
{
    return cv::Point((corners[0] + corners[2]) / 2);
}