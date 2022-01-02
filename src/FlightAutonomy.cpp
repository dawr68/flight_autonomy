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
    if (!flightCtrl.connect())
    {
        return false;
    }

    if (!flightCtrl.startOffbard())
    {
        return false;
    }

    return true;
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
    cv::Mat img = imgRec.getImage();

    if (objDetect.detect(img))
    {
        if(flightCtrl.getAltitude() < 0.8)
        {
            flightCtrl.land();
            return;
        }
        cv::Vec3f arPos = objDetect.getPosition();
        cv::Vec3f halfFrameSize(img.cols / 2, img.rows / 2, 0.f);
        cv::Vec3f normalVec = (arPos - halfFrameSize);
        normalVec = cv::Vec3f(-normalVec[1] / halfFrameSize[1], normalVec[0] / halfFrameSize[0], cv::abs(normalVec[2]));
        
        if(pow(normalVec[0], 2) + pow(normalVec[1], 2) < 0.5)
        {
            normalVec[2] = 0.5f;
        }
        std::cout << normalVec << std::endl;

        flightCtrl.setOffbardVelo({normalVec[0] * 1 , normalVec[1] * 1, normalVec[2], 0.0f});
    }
    else
    {
        flightCtrl.setOffbardVelo({0.f, 0.f, 0.f, 0.f});
    }



#ifdef FA_DEBUG
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(1);
#endif

    flightCtrl.printTelem();
}