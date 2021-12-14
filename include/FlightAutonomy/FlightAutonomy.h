#pragma once

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>

#include "FlightAutonomy/ImageReceiver.h"
#include "FlightAutonomy/FlightControl.h"

#define FA_DEBUG

/**
 * @brief Class responsible for
 *
 */
class FlightAutonomy
{
#ifdef FA_DEBUG
    const std::string OPENCV_WINDOW = "Cam View";
#endif

    ros::NodeHandle &nh;
    ImageReceiver imgRec;
    FlightControl flightCtrl;

public:
    FlightAutonomy(ros::NodeHandle &);
    ~FlightAutonomy();

    bool connect();
    void readArgs(const int argc, char **argv);
    void spinOnce();

private:
    std::string getArg(const std::string option);
};