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
            center = calcCenter(markCor[i]);
        }
    }

    return center;
}

std::tuple<cv::Point2f, int> ObjectDetector::detectArucoGate(const cv::Mat &img, int arucoIDs[4])
{
    cv::Point2f center(-1., -1.);
    int size = -1;

    cv::aruco::detectMarkers(img, dict, markCor, markIds, params, rejected);

#ifdef FA_DEBUG
    cv::aruco::drawDetectedMarkers(img, markCor, markIds);
#endif
    std::vector<cv::Point2f> gateVertices;

    for (int j = 0; j < 4; j++)
    {
        for (int i = 0; i < markIds.size(); i++)
        {
            if (markIds[i] == arucoIDs[gateVertices.size()])
            {
                gateVertices.push_back(calcCenter(markCor[i]));
            }
        }
    }

    if (gateVertices.size() == 4)
    {
        center = calcCenter(gateVertices);
        size = abs(gateVertices[0].x - gateVertices[1].x); 
    }

    return std::make_tuple(center, size);
}

cv::Point2f ObjectDetector::calcCenter(std::vector<cv::Point2f> corners)
{
    return cv::Point((corners[0] + corners[2]) / 2);
}