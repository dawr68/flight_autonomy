#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/aruco.hpp>

/**
 * @brief Klasa odpowiedzialna za analizę i wykrywanie oraz określanie pozycji obiektów (w tym znaczników Aruco) znajdujących się na przekazanym obrazie.
 * 
 */
class ObjectDetector
{
    int arucoId = 68;

    std::vector<int> markIds;
    std::vector<std::vector<cv::Point2f>> markCor, rejected;

    cv::Ptr<cv::aruco::DetectorParameters> params;
    cv::Ptr<cv::aruco::Dictionary> dict;

    cv::Vec3f objPostion;

public:
    ObjectDetector();
    ObjectDetector(cv::Ptr<cv::aruco::DetectorParameters> _params, cv::Ptr<cv::aruco::Dictionary> _dict);
    ~ObjectDetector() = default;

    bool detect(const cv::Mat &img);

    cv::Vec3f getPosition();
};

cv::Point2i calcArucoCenter(std::vector<cv::Point2f> corners);