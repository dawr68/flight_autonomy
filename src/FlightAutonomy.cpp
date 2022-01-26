#include "FlightAutonomy/FlightAutonomy.h"

FlightAutonomy::FlightAutonomy(ros::NodeHandle &_nh) : imgRec(_nh, "/iris_race/c920/image_raw")
{
    exitCode = 0;
    timeoutCounter = std::chrono::steady_clock::now();
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
    // if (!imgRec.open())
    // {
    //     std::cerr << "[ERROR] Image stream opening error" << std::endl;
    //     return false;
    // }

    if (!flightCtrl.connect())
    {
        std::cerr << "[ERROR] Autopilot connection error" << std::endl;
        return false;
    }

    if (!flightCtrl.startOffbard())
    {
        std::cerr << "[ERROR] Could not start offboard" << std::endl;
        return false;
    }

    return true;
}

bool FlightAutonomy::isReady()
{
    int i = 0;
    for (; i < 10 && !flightCtrl.observeInAir(); i++)
    {
        std::cout << "Waiting for drone to be in air..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (i > 9)
    {
        return false;
    }

    return flightCtrl.checkStatus();
}

bool FlightAutonomy::ok()
{
    if (exitCode)
    {
        return false;
    }
    else
    {
        if (flightCtrl.getAltitude() > 10.)
        {
            exitCode = 2;
            return false;
        }
        else
        {
            return flightCtrl.checkStatus();
        }
    }
}

bool FlightAutonomy::stop()
{
    return flightCtrl.stopOffboard();
}

int FlightAutonomy::getExitCode()
{
    return exitCode;
}

void FlightAutonomy::printExitStatus()
{
    switch (exitCode)
    {
    case 0:
        std::cout << "Exitting due to error..." << std::endl;
        break;

    case 1:
        std::cout << "Normal exit" << std::endl;
        break;

    case 2:
        std::cout << "Altitude too high" << std::endl;
        break;

    case 3:
        std::cout << "No markers detected for " << NO_DETECT_TIMEOUT << "s" << std::endl;
        break;

    default:
        break;
    }
}

bool FlightAutonomy::readArgs(const int argc, char **argv)
{
    if (argc < 3 || argc > 4)
    {
        return false;
    }

    if (!std::strcmp(argv[1], "landing"))
    {
        currAlg = arucoLanding;
    }
    else
    {
        if (!std::strcmp(argv[1], "racing"))
        {
            currAlg = gateRacing;
        }
        else
        {
            return false;
        }
    }

    flightCtrl.setConnectionURL(argv[2]);

    if (argc == 4 && std::strlen(argv[4]) == 1 && std::isdigit(argv[4][0]))
    {
        imgRec.setDevice((int)argv[4][0]);
    }

    return true;
}

bool FlightAutonomy::landingStep(cv::Mat &img)
{
    mavsdk::Offboard::VelocityBodyYawspeed velo = {};

    if (flightCtrl.getAltitude() < LAND_ALT)
    {
        flightCtrl.setOffbardVelo(velo);
        flightCtrl.land();
        currAlg = normalLanding;
        return true;
    }

    cv::Point2f arPos = objDetect.detectArucoSingle(img, landingPadID);

    if (arPos != cv::Point2f(-1, -1))
    {
        cv::Point2f halfFrameSize(img.cols / 2.f, img.rows / 2.f);

        cv::Point2f normalVec;
        normalVec.x = (arPos.x - halfFrameSize.x) / halfFrameSize.x;
        normalVec.y = (arPos.y - halfFrameSize.y) / halfFrameSize.y;

        if (pow(normalVec.x, 2) + pow(normalVec.y, 2) < MID_THRESHOLD)
        {
            velo.down_m_s = MAX_VELO_VERT_MS;
        }

        // Drone lean compensation
        mavsdk::Telemetry::EulerAngle droneAngles = flightCtrl.getEulerAngle();
        normalVec.x -= droneAngles.pitch_deg / (0.5 * CAM_VFOV);
        normalVec.y -= droneAngles.roll_deg / (0.5 * CAM_HFOV);

        velo.forward_m_s = -normalVec.y * MAX_VELO_HORI_MS;
        velo.right_m_s = normalVec.x * MAX_VELO_HORI_MS;
    }
    else
    {
        return false;
    }

    flightCtrl.setOffbardVelo(velo);

    return true;
}

bool FlightAutonomy::avoidingStep(cv::Mat &img)
{
    int gateArucos[] = {10, 11, 12, 13};
    bool returnCode = true;
    cv::Point2f halfFrameSize(img.cols / 2.f, img.rows / 2.f);
    mavsdk::Offboard::VelocityBodyYawspeed velo = {};

    std::tuple<cv::Point2f, float, float> gatePosSizeAng = objDetect.detectArucoGate(img, gateArucos);

    cv::Point2f gatePos = std::get<0>(gatePosSizeAng);
    float size = std::get<1>(gatePosSizeAng);
    float angle = std::get<2>(gatePosSizeAng) * 180 / CV_PI;

    if (size > 0.45)
    {
        velo.forward_m_s = 1 * MAX_VELO_HORI_MS;
        flightCtrl.setOffbardVelo(velo);
        currAlg = forwardFlight;
        returnCode = true;
    }

    if (gatePos != cv::Point2f(-1, -1))
    {
#ifdef FA_DEBUG
        cv::circle(img, gatePos, 10, cv::Scalar(0, 0, 255), 4);
#endif

        cv::Point2f normalVec;
        normalVec.x = (gatePos.x - halfFrameSize.x) / halfFrameSize.x;
        normalVec.y = (gatePos.y - halfFrameSize.y) / halfFrameSize.y;

        // // Drone lean compensation
        mavsdk::Telemetry::EulerAngle droneAngles = flightCtrl.getEulerAngle();
        angle = angle + droneAngles.roll_deg;

        if (angle < 1.25)
        {
            velo.forward_m_s = 0.25 * MAX_VELO_HORI_MS;
        } else {
            velo.forward_m_s = 0.05 * MAX_VELO_HORI_MS;
        }
        velo.yawspeed_deg_s = normalVec.x * MAX_YAWSPEED;
        velo.right_m_s = angle * 0.075 * MAX_VELO_HORI_MS;
    }
    else
    {
        velo.yawspeed_deg_s = -0.2 * MAX_YAWSPEED;
        returnCode = false;
    }

    float currAlt = flightCtrl.getAltitude();
    if (currAlt < 0.9)
    {
        velo.down_m_s = -0.2;
    }
    else
    {
        if (currAlt > 1.1)
        {
            velo.down_m_s = 0.2;
        }
    }

    flightCtrl.setOffbardVelo(velo);

    return returnCode;
}

bool FlightAutonomy::spinOnce()
{
    // imgRec.receiveImage();
    cv::Mat img = imgRec.getImage();

    bool returnCode;

    switch (currAlg)
    {
    case arucoLanding:
    {
        returnCode = landingStep(img);
        break;
    }
    case gateRacing:
    {
        returnCode = avoidingStep(img);
        break;
    }
    case normalLanding:
    {
        if (!flightCtrl.observeInAir())
        {
            exitCode = 1;
        }
        break;
    }
    case forwardFlight:
    {
        mavsdk::Offboard::VelocityBodyYawspeed velo = {0.5f * MAX_VELO_HORI_MS, 0.f, 0.f, 0.f};
        returnCode = 0;
        float currAlt = flightCtrl.getAltitude();
        if (currAlt < 0.9)
        {
            velo.down_m_s = -0.2;
        }
        else
        {
            if (currAlt > 1.1)
            {
                velo.down_m_s = 0.2;
            }
        }
        flightCtrl.setOffbardVelo(velo);
        break;
    }

    default:
    {
        break;
    }
    }

    std::chrono::steady_clock::time_point tNow = std::chrono::steady_clock::now();
    if (returnCode == 0)
    {
        if (currAlg != forwardFlight)
        {
            if (std::chrono::duration_cast<std::chrono::seconds>(tNow - timeoutCounter).count() >= NO_DETECT_TIMEOUT)
            {
                exitCode = 3;
            }
        }
        else
        {
            if (std::chrono::duration_cast<std::chrono::seconds>(tNow - timeoutCounter).count() >= FORWARD_FLIGHT_TIMEOUT)
            {
                currAlg = gateRacing;
            }
        }
    }
    else
    {
        timeoutCounter = std::chrono::steady_clock::now();
    }

#ifdef FA_DEBUG
    cv::resize(img, img, cv::Size(320, 240));
    cv::imshow(OPENCV_WINDOW, imgRec.getImage());
    cv::waitKey(1);
#endif

    // flightCtrl.printTelem();

    return returnCode;
}