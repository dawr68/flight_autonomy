#include <ros/ros.h>
#include "FlightAutonomy/FlightAutonomy.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "flight_autonomy");
    ros::NodeHandle nh;
    ros::Rate rate(30);

    FlightAutonomy fa(nh);

    fa.connect();
    
    while (ros::ok())
    {
        ros::spinOnce();
        fa.spinOnce();
        rate.sleep();
    }

    return 0;
}