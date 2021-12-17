#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/aruco.hpp>

/**
 * @brief Klasa odpowiedzialna za analizę obrazu i wykrywanie obiektów (w tym znaczników Aruco) znajdujących się na nim.
 * 
 */
class ObjectDetector
{
    int arucoId = 68;

    std::vector<int> markIds;
    std::vector<std::vector<cv::Point2f>> markCor, rejected;

    cv::Ptr<cv::aruco::DetectorParameters> params;
    cv::Ptr<cv::aruco::Dictionary> dict;

public:
    ObjectDetector();
    ObjectDetector(cv::Ptr<cv::aruco::DetectorParameters> _params, cv::Ptr<cv::aruco::Dictionary> _dict);
    ~ObjectDetector() = default;

    bool detect(const cv::Mat &img);
};