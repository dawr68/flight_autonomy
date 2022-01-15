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

cv::Point2f ObjectDetector::detectArucoSingle(const cv::Mat &img, int arucoID)
{
    cv::Point2f center(-1., -1.);

    cv::aruco::detectMarkers(img, dict, markCor, markIds, params, rejected);

#ifdef FA_DEBUG
    cv::aruco::drawDetectedMarkers(img, markCor, markIds);
#endif

    for (int i = 0; i < markIds.size(); i++)
    {
        if (markIds[i] == arucoID)
        {
            center = calcArucoCenter(markCor[i]);
        }
    }

    return center;
}

// cv::Point2i ObjectDetector::detectGateSingle(const cv::Mat &img, int arucoIDs[4])
// {
//     cv::aruco::detectMarkers(img, dict, markCor, markIds, params, rejected);

// #ifdef FA_DEBUG
//     cv::aruco::drawDetectedMarkers(img, markCor, markIds);
// #endif

//     if (markIds.size() == 0)
//     {
//         objPostion = cv::Point2i(-1, -1);
//         return 0;
//     }

//     for (auto mCor : markCor)
//     {
//         objPostion = calcArucoCenter(mCor);
//     }

//     return 1;
// }

cv::Point2f ObjectDetector::calcArucoCenter(std::vector<cv::Point2f> corners)
{
    return cv::Point((corners[0] + corners[2]) / 2);
}

cv::Point2f ObjectDetector::calcGateCenter(std::vector<cv::Point2f> corners)
{
    return cv::Point((corners[0] + corners[2]) / 2);
}