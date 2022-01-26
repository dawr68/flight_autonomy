#include <ros/ros.h>
#include "FlightAutonomy/FlightAutonomy.h"

void printHelp(std::string progName)
{
    std::cout << "Usage:" << std::endl;
    std::cout << progName + " algorithm drone_connection_URL [cameraID]" << std::endl;
    std::cout << "For example" << std::endl;
    std::cout << progName + "landing udp://:14445 0" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "flight_autonomy");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    FlightAutonomy fa(nh);

    if (!fa.readArgs(argc, argv))
    {
        printHelp(argv[0]);
        return 0;
    }

    if (!fa.connect())
    {
        return 1;
    }

    if (!fa.isReady())
    {
        std::cerr << "Drone not ready, exiting..." << std::endl;
        return 2;
    }

    while (fa.ok())
    {
        ros::spinOnce();
        fa.spinOnce();
        rate.sleep();
    }

    fa.stop();
    fa.printExitStatus();

    std::cout << "Exitted successfully." << std::endl;

    return 0;
}