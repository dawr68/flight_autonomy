#include "FlightAutonomy/FlightAutonomy.h"

void printHelp(std::string progName)
{
    std::cout << "Usage:" << std::endl;
    std::cout << "  " << progName + " algorithm drone_connection_URL [cameraID]" << std::endl;
    std::cout << "For example " << std::endl;
    std::cout << "  " << progName + " landing udp://:14445 0" << std::endl;
    std::cout << "Available algorithms:" << std::endl;
    std::cout << "   landing / racing" << std::endl;
}

int main(int argc, char **argv)
{
    FlightAutonomy fa;

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
        fa.spinOnce();
    }

    fa.stop();
    fa.printExitStatus();

    std::cout << "Exitted successfully." << std::endl;

    return 0;
}