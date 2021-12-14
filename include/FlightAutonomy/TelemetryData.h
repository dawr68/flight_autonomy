#pragma once

#include <mavsdk/plugins/telemetry/telemetry.h>

struct TelemetryData
{
    uint64_t UUID = 0;
    bool health = false;

    bool isArmed = false;
    bool inAir = false;

    float batteryPercent = -1;

    mavsdk::Telemetry::Odometry odom;
};