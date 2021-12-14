#pragma once

#include <future>
#include <chrono>
#include <iostream>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/info/info.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>

#include "TelemetryData.h"

class FlightControl
{
    std::string connectionURL;

    mavsdk::Mavsdk mavsdk;
    std::shared_ptr<mavsdk::System> system;
    std::shared_ptr<mavsdk::Info> info;
    std::shared_ptr<mavsdk::Telemetry> telemetry;
    std::shared_ptr<mavsdk::Action> action;
    std::shared_ptr<mavsdk::Offboard> offboard;

    TelemetryData telemData;

    std::shared_ptr<mavsdk::System> getSystem();
    void subscribeTelem();

public:

    FlightControl();
    ~FlightControl();

    bool connect();

    void setConnectionURL(const std::string url);

    void printTelem();
};