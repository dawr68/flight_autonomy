#include "FlightAutonomy/FlightAutonomy.h"

FlightAutonomy::FlightAutonomy(ros::NodeHandle &_nh) : nh(_nh), imgRec(_nh, "/iris_race/c920/image_raw")
{
#ifdef FA_DEBUG
    cv::namedWindow(OPENCV_WINDOW);
#endif
}

FlightAutonomy::~FlightAutonomy()
{
    cv::destroyAllWindows();
}