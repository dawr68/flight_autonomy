#include "FlightAutonomy/FlightControl.h"

void FlightControl::subscribeTelem()
{
    telemetry->subscribe_health([this](mavsdk::Telemetry::Health health)
                                {
        if (health.is_local_position_ok && health.is_armable)
            this->telemData.health = true; });

    telemetry->subscribe_armed([this](bool armed)
                               { this->telemData.isArmed = armed; });

    telemetry->subscribe_in_air([this](bool inAir)
                                { this->telemData.inAir = inAir; });

    telemetry->subscribe_battery([this](mavsdk::Telemetry::Battery bat)
                                 { this->telemData.batteryPercent = bat.remaining_percent; });

    telemetry->subscribe_position([this](mavsdk::Telemetry::Position pos)
                                  { this->telemData.altitude = pos.relative_altitude_m; });

    telemetry->subscribe_attitude_euler([this](mavsdk::Telemetry::EulerAngle eur)
                                        { this->telemData.eulerAngle = eur; });

    telemetry->subscribe_odometry([this](mavsdk::Telemetry::Odometry odo)
                                  { this->telemData.odom = odo; });
}

bool FlightControl::connect()
{
    mavsdk::ConnectionResult conRes = mavsdk.add_any_connection(connectionURL);

    if (conRes != mavsdk::ConnectionResult::Success)
    {
        return false;
    }

    system = getSystem();
    if (!system)
    {
        return false;
    }

    info = std::make_shared<mavsdk::Info>(system);
    telemetry = std::make_shared<mavsdk::Telemetry>(system);
    action = std::make_shared<mavsdk::Action>(system);
    offboard = std::make_shared<mavsdk::Offboard>(system);

    while (!telemetry->health_all_ok())
    {
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    subscribeTelem();

    return true;
}

std::shared_ptr<mavsdk::System> FlightControl::getSystem()
{
    auto promise = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto future = promise.get_future();

    mavsdk.subscribe_on_new_system([this, &promise]()
                                   {
        auto system = mavsdk.systems().back();
        if(system->has_autopilot()) {
            mavsdk.subscribe_on_new_system(nullptr);
            promise.set_value(system);
        } });

    if (future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    {
        return {};
    }

    return future.get();
}

void FlightControl::setConnectionURL(const std::string url)
{
    connectionURL = url;
}

void FlightControl::printTelem()
{
    std::cout << telemData.isArmed << std::endl;
    std::cout << telemData.batteryPercent << std::endl;
    std::cout << telemData.eulerAngle << std::endl;
    std::cout << "\n";
}

bool FlightControl::startOffbard()
{
    offboard->set_velocity_body({0.0f, 0.0f, 0.0f, 0.0f});
    mavsdk::Offboard::Result startResult = offboard->start();
    if (startResult != mavsdk::Offboard::Result::Success)
    {
        return false;
    }

    return true;
}

bool FlightControl::stopOffboard()
{
    mavsdk::Offboard::Result stopResult = offboard->stop();
    if (stopResult != mavsdk::Offboard::Result::Success)
    {
        return false;
    }

    return true;
}

bool FlightControl::setOffbardVelo(mavsdk::Offboard::VelocityBodyYawspeed veloBodyYawspeed)
{
    if (offboard->is_active())
    {
        offboard->set_velocity_body(veloBodyYawspeed);
    }
    else
    {
        return 0;
    }

    return 1;
}

bool FlightControl::land()
{
    if (action->land() == mavsdk::Action::Result::Success)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

float FlightControl::getAltitude()
{
    return telemData.altitude;
}