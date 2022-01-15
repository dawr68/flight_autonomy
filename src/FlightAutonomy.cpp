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
        std::cerr << "[ERROR] Autopilot connection error" << std::endl;
        return false;
    }

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
    for (; i < 10, !flightCtrl.observeInAir(); i++)
    {
        std::cout << "Waiting for drone to be in air..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    if (i == 9)
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

    if (flightCtrl.getAltitude() < 0.8)
    {
        flightCtrl.setOffbardVelo(velo);
        flightCtrl.land();
        exitCode = 1;
        return true;
    }

    cv::Point2f arPos = objDetect.detectArucoSingle(img, landingPadID);

    if (arPos != cv::Point2f(-1, -1))
    {
        cv::Point2f halfFrameSize(img.cols / 2.f, img.rows / 2.f);
        // Normalized vector
        cv::Point2f normalVec;
        normalVec.x = (arPos.x - halfFrameSize.x) / halfFrameSize.x;
        normalVec.y = (arPos.y - halfFrameSize.y) / halfFrameSize.y;

        std::cout << "Normal vec" << normalVec << std::endl;
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
        std::cout << "VELO: " << velo << std::endl;
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
    return true;
}

bool FlightAutonomy::spinOnce()
{
    imgRec.receiveImage();
    cv::Mat img = imgRec.getImage();

    bool returnCode;

    switch (currAlg)
    {
    case arucoLanding:
        returnCode = landingStep(img);
        break;

    case gateRacing:
        returnCode = avoidingStep(img);
        break;

    case normalLanding:
        if (!flightCtrl.observeInAir())
        {
            exitCode = 1;
        }
        break;

    default:
        break;
    }

    if (returnCode == 0)
    {
        std::chrono::steady_clock::time_point tNow = std::chrono::steady_clock::now();
        if (std::chrono::duration_cast<std::chrono::seconds>(tNow - timeoutCounter).count() >= NO_DETECT_TIMEOUT)
        {
            exitCode = 3;
        }
    }
    else
    {
        timeoutCounter = std::chrono::steady_clock::now();
    }

#ifdef FA_DEBUG
    cv::imshow(OPENCV_WINDOW, imgRec.getImage());
    cv::waitKey(1);
#endif

    // flightCtrl.printTelem();

    return returnCode;
}