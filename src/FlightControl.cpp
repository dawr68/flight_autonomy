#include "FlightAutonomy/FlightControl.h"

FlightControl::FlightControl()
{
}

FlightControl::~FlightControl()
{
}

bool FlightControl::connect(const std::string connectionUrl)
{
        std::cout << "connecting";
    mavsdk::ConnectionResult conRes= mavsdk.add_any_connection(connectionUrl);

    if(conRes != mavsdk::ConnectionResult::Success)
    {
        return 1;
    }

    system = getSystem();
    if(!system)
    {
        return 1;
    }

    info = std::make_shared<mavsdk::Info>(system);
    telemetry = std::make_shared<mavsdk::Telemetry>(system);
    action = std::make_shared<mavsdk::Action>(system);
    offboard = std::make_shared<mavsdk::Offboard>(system);

    
    while(!telemetry->health_all_ok())
    {
        std::this_thread::sleep_for(std::chrono::seconds(3));
    }

    return 0;

}

std::shared_ptr<mavsdk::System> FlightControl::getSystem()
{
    auto promise = std::promise<std::shared_ptr<mavsdk::System>>{};
    auto future = promise.get_future();

    mavsdk.subscribe_on_new_system([this, &promise]() {
        auto system = mavsdk.systems().back();
        if(system->has_autopilot()) {
            mavsdk.subscribe_on_new_system(nullptr);
            promise.set_value(system);
        }
    });

    if(future.wait_for(std::chrono::seconds(3)) == std::future_status::timeout)
    {
        return {};
    }

    return future.get();
}
