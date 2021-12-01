#pragma once

#include <ros/ros.h>

#include "FlightAutonomy/ImageReceiver.h"

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

public:
    FlightAutonomy(ros::NodeHandle &);
    ~FlightAutonomy();

};