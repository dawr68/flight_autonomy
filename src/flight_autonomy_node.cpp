#include <ros/ros.h>
#include "FlightAutonomy/FlightAutonomy.h"

void printHelp(std::string progName)
{
    std::cerr << "Usage: " + progName + " <drone_connection_URL>" << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printHelp(argv[0]);
        return 0;
    }

    ros::init(argc, argv, "flight_autonomy");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    FlightAutonomy fa(nh);
    
    fa.readArgs(argc, argv);

    if (fa.connect())
    {
        std::cerr << "Autopilot connection error" << std::endl;
        return 1;
    }

    while (ros::ok())
    {
        ros::spinOnce();
        fa.spinOnce();
        rate.sleep();
    }

    return 0;
}