#include "FlightAutonomy/FlightAutonomy.h"

FlightAutonomy::FlightAutonomy(ros::NodeHandle &_nh) : nh(_nh), imgRec(_nh, "/iris_race/c920/image_raw"), flightCtrl()
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
    if (flightCtrl.connect())
    {
        return true;
    }

    return false;
}

void FlightAutonomy::readArgs(const int argc, char **argv)
{
    if (argc >= 2)
    {
        flightCtrl.setConnectionURL(argv[1]);
    }
}

void FlightAutonomy::spinOnce()
{
#ifdef FA_DEBUG
    cv::Mat img = imgRec.getImage();
    cv::imshow(OPENCV_WINDOW, imgRec.getImage());
    cv::waitKey(1);
#endif

}
