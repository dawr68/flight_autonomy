#include "FlightAutonomy/FlightAutonomy.h"

FlightAutonomy::FlightAutonomy(ros::NodeHandle &_nh) : nh(_nh), imgRec(_nh, "/iris_race/c920/image_raw"), flCtrl()
{
#ifdef FA_DEBUG
    cv::namedWindow(OPENCV_WINDOW);
#endif
}

FlightAutonomy::~FlightAutonomy()
{
#ifdef FA_DEBUG
    cv::destroyAllWindows();
#endif
}

bool FlightAutonomy::connect()
{
    if(flCtrl.connect("udp://:14445"))
    {
        return false;
    }

    return true;
}

void FlightAutonomy::readArgs(int argc, char **argv)
{
}

void FlightAutonomy::spinOnce()
{
#ifdef FA_DEBUG
    cv::Mat img = imgRec.getImage();
    cv::imshow(OPENCV_WINDOW, imgRec.getImage());
    cv::waitKey(1);
#endif
}
