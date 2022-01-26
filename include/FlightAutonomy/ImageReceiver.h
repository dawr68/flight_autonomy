#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio.hpp>

/**
 * @brief Klasa odbierająca stream wideo z podanego źródła.
 */
class ImageReceiver
{
    ros::NodeHandle &nh;     /**< Uchwyt do węzła ROS */
    ros::Subscriber imgSub;  /**< Subskrybent do obrazu z kamery */
    cv::VideoCapture cap;    /**< Obiekt przechwytujący stream video */
    int deviceID = 0;        /**< Kamera 0 - domyślna */
    int apiID = cv::CAP_ANY; /**< Domyślnie autodetekcja */
    cv::Mat camImage;        /**< Najnowsza ramka odebrana z kamery*/

public:
    /**
     * @brief Konstruuje obiekt ImageReceiver.
     */
    ImageReceiver() = default;

    /**
     * @brief Konstruuje obiekt ImageReceiver.
     */
    ImageReceiver(ros::NodeHandle &_nh, std::string topic);

    /**
     * @brief Konstruuje obiekt ImageReceiver wpisując przekazane wartości do pól tworzonego obiektu.
     */
    ImageReceiver(int deviceID, ros::NodeHandle &_nh, int apiID = 0);

    /**
     * @brief Niszczy obiekt ImageReceiver.
     */
    ~ImageReceiver() = default;

    /**
     * @brief Otwiera wejście kamery
     *
     * @return true Pomyślnie otwarto wejście kamery.
     * @return false Wystąpił błąd podczas otwierania wejścia kamery.
     */
    bool open();

    /**
     * @brief Pobiera ramkę z wejścia kamery
     */
    void receiveImage();

    /**
     * @brief Set the Device object
     *
     * @param _deviceID
     * @param _apiID
     */
    void setDevice(int _deviceID, int _apiID = 0);

    /**
     * @brief Zwraca najnowszą odebraną ramkę obrazu.
     *
     * @return Ramka obrazu.
     */
    cv::Mat getImage();

    /**
     * @brief Wywołanie zwrotne po pojawieniu się nowej ramki obrazu.
     * 
     * @param img Wskaźnik na najnowszą ramkę
     */
    void imgSubCallback(const sensor_msgs::ImageConstPtr &img);
};