#include "FlightAutonomy/FlightAutonomy.h"

FlightAutonomy::FlightAutonomy()
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
    if (!imgRec.open())
    {
        std::cerr << "[ERROR] Image input open error" << std::endl;
        return false;
    }

    if (!flightCtrl.connect())
    {
        std::cerr << "[ERROR] Autopilot connection error" << std::endl;
        return false;
    }

    if (!flightCtrl.startOffbard())
    {
        std::cerr << "[ERROR] Can not start offboard" << std::endl;
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

    if (i >= 10)
    {
        return false;
    }

    i = 0;
    for (; i < 5 && flightCtrl.getFlightMode() != mavsdk::Telemetry::FlightMode::Offboard; i++)
    {
        std::cout << "Waiting for drone to change FM to offboard" << std::endl;
        mavsdk::Offboard::VelocityBodyYawspeed velo = {};
        flightCtrl.setOffbardVelo(velo);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    if (i >= 5)
    {
        exitCode = 4;
        return false;
    }

    return flightCtrl.checkStatus();
}

bool FlightAutonomy::ok()
{
    if (exitCode || !(flightCtrl.getFlightMode() != mavsdk::Telemetry::FlightMode::Offboard || flightCtrl.getFlightMode() != mavsdk::Telemetry::FlightMode::Land))
    {
        return false;
    }
    else
    {
        if (flightCtrl.getAltitude() > MAX_FLIGHT_ALT)
        {
            exitCode = 2;
            return false;
        }
        else
        {
            return flightCtrl.checkStatus();
        }
    }
    return true;
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

    case 4:
        std::cout << "Offboard FM mode not enabled" << std::endl;
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

    if (argc == 4 && std::strlen(argv[3]) == 1 && std::isdigit(argv[3][0]))
    {
        imgRec.setDevice(atoi(argv[3]));
    }

    return true;
}

bool FlightAutonomy::landingStep(cv::Mat &img)
{
    bool returnCode = false;
    mavsdk::Offboard::VelocityBodyYawspeed velo = {};

    if (flightCtrl.getAltitude() < LAND_ALT)
    {
        flightCtrl.setOffbardVelo(velo);
        flightCtrl.land();
        currAlg = normalLanding;
        returnCode = false;
    }

    cv::Point2f arPos = objDetect.detectArucoSingle(img, landingPadID);

    if (arPos != cv::Point2f(-1, -1))
    {
        velo = landingCalcVelo(img, arPos);
#ifdef FA_DEBUG
        cv::circle(img, arPos, 10, cv::Scalar(0, 0, 255), 4);
#endif
        returnCode = true;
    }

    flightCtrl.checkVelo(velo);
    flightCtrl.setOffbardVelo(velo);

    return returnCode;
}

mavsdk::Offboard::VelocityBodyYawspeed FlightAutonomy::landingCalcVelo(cv::Mat &img, cv::Point2f arucoPosition)
{
    mavsdk::Offboard::VelocityBodyYawspeed velo = {};
    cv::Point2f halfFrameSize(img.cols / 2.f, img.rows / 2.f);

    cv::Point2f dispVec;
    dispVec.x = (arucoPosition.x - halfFrameSize.x) / halfFrameSize.x;
    dispVec.y = (arucoPosition.y - halfFrameSize.y) / halfFrameSize.y;

    if (pow(dispVec.x, 2) + pow(dispVec.y, 2) < MID_THRESHOLD)
    {
        velo.down_m_s = MAX_VELO_VERT_MS;
    }

    // Drone lean compensation
    mavsdk::Telemetry::EulerAngle droneAngles = flightCtrl.getEulerAngle();
    dispVec.x -= droneAngles.pitch_deg / (0.5 * CAM_VFOV);
    dispVec.y -= droneAngles.roll_deg / (0.5 * CAM_HFOV);

    velo.forward_m_s = -dispVec.y * MAX_VELO_HORI_MS;
    velo.right_m_s = dispVec.x * MAX_VELO_HORI_MS;
    return velo;
}

bool FlightAutonomy::avoidingStep(cv::Mat &img)
{
    bool returnCode = true;
    mavsdk::Offboard::VelocityBodyYawspeed velo = {};
    std::tuple<cv::Point2f, float, float> gatePosSizeAng = objDetect.detectArucoGate(img, gateArucos);

    cv::Point2f gatePos = std::get<0>(gatePosSizeAng);
    float size = std::get<1>(gatePosSizeAng);
    float angle = std::get<2>(gatePosSizeAng) * 180 / CV_PI;

    if (size > 0.45)
    {
        velo.forward_m_s = 1 * MAX_VELO_HORI_MS;
        currAlg = forwardFlight;
        returnCode = true;
    }
    else
    {
        if (gatePos != cv::Point2f(-1, -1))
        {
            velo = avoidingCalcVelo(img, gatePos, angle);
#ifdef FA_DEBUG
            cv::circle(img, gatePos, 10, cv::Scalar(0, 0, 255), 4);
#endif
        }
        else
        {
            velo.yawspeed_deg_s = -0.2 * MAX_YAWSPEED;
            returnCode = false;
        }
    }

    velo.down_m_s = flightCtrl.getAltitude() - RACING_ALT;

    flightCtrl.checkVelo(velo);
    flightCtrl.setOffbardVelo(velo);

    return returnCode;
}

mavsdk::Offboard::VelocityBodyYawspeed FlightAutonomy::avoidingCalcVelo(cv::Mat &img, cv::Point2f gatePosition, float angle)
{
    cv::Point2f halfFrameSize(img.cols / 2.f, img.rows / 2.f);
    mavsdk::Offboard::VelocityBodyYawspeed velo = {};

    cv::Point2f dispVec;
    dispVec.x = (gatePosition.x - halfFrameSize.x) / halfFrameSize.x;
    dispVec.y = (gatePosition.y - halfFrameSize.y) / halfFrameSize.y;

    // Drone lean compensation
    mavsdk::Telemetry::EulerAngle droneAngles = flightCtrl.getEulerAngle();
    angle = angle + droneAngles.roll_deg;

    velo.forward_m_s = 0.05 * MAX_VELO_HORI_MS;

    if (angle < IS_STRAIGHT_ANGLE)
    {
        velo.forward_m_s = 0.25 * MAX_VELO_HORI_MS;
    }
    velo.yawspeed_deg_s = dispVec.x * MAX_YAWSPEED;
    velo.right_m_s = angle * 0.075 * MAX_VELO_HORI_MS;

    return velo;
}

void FlightAutonomy::checkTimeouts()
{
    std::chrono::steady_clock::time_point tNow = std::chrono::steady_clock::now();
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

int FlightAutonomy::performStep(cv::Mat &img)
{
    int returnCode;

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
        mavsdk::Offboard::VelocityBodyYawspeed velo = {0.5f * MAX_VELO_HORI_MS, 0.f, (flightCtrl.getAltitude() - RACING_ALT), 0.f};
        returnCode = 0;
        flightCtrl.checkVelo(velo);
        flightCtrl.setOffbardVelo(velo);
        break;
    }

    default:
    {
        break;
    }
    }

    return returnCode;
}

bool FlightAutonomy::spinOnce()
{
    imgRec.receiveImage();
    cv::Mat img = imgRec.getImage();

    bool returnCode = performStep(img);

    if (returnCode == 0)
    {
        checkTimeouts();
    }
    else
    {
        timeoutCounter = std::chrono::steady_clock::now();
    }

#ifdef FA_DEBUG
    cv::resize(img, img, cv::Size(640, 480));
    cv::imshow(OPENCV_WINDOW, img);
    cv::waitKey(1);
#endif

    return 0;
}