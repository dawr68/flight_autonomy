#pragma once

#include <mavsdk/plugins/telemetry/telemetry.h>

/**
 * @brief  Struktura przechowująca dane telemetryczne odebrane z pojazdu przez MavLink.
 */
struct TelemetryData
{
    uint64_t UUID = 0;                        ///< Identyfikator systemu.
    bool health = false;                      ///< Stan pojazdu. Domyślnie niegotowy do lotu.
    mavsdk::Telemetry::FlightMode flightMode; ///< Tryb lotu.

    bool isArmed = false; ///< Stan uzbrojenia. Domyślnie rozbrojony.
    bool inAir = false;   ///< Czy pojazd jest w powietrzu.

    float batteryPercent = -1; ///< Stan naładowania akumulatorów.

    float altitude = 0; ///< Wysokość relatywna względem miejsca startu.

    mavsdk::Telemetry::EulerAngle eulerAngle; ///< Aktualne kąty pochylenia autopilota.
    mavsdk::Telemetry::Odometry odom;         ///< Aktualne dane otometryczne.
};